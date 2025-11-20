#include "video_pipeline.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <gtk/gtk.h>
#include <algorithm>
#include <json/json.h>
#include <fstream>
#include <filesystem>

// Helper to cleanup recording bin on the main thread
static gboolean cleanup_recording_bin(gpointer data)
{
    GstElement* bin = static_cast<GstElement*>(data);
    if (bin) {
        gst_element_set_state(bin, GST_STATE_NULL);
        GstObject* parent = gst_element_get_parent(bin);
        if (parent) {
            gst_bin_remove(GST_BIN(parent), bin);
            gst_object_unref(parent);
        }
        // bin is unref'd by gst_bin_remove usually, but we might hold a ref if we created it?
        // gst_bin_add takes ownership.
    }
    return G_SOURCE_REMOVE;
}

// Probe to detect EOS at the filesink
static GstPadProbeReturn eos_cb(GstPad* pad, GstPadProbeInfo* info, gpointer user_data)
{
    GstEvent* event = GST_PAD_PROBE_INFO_EVENT(info);
    if (GST_EVENT_TYPE(event) == GST_EVENT_EOS) {
        GstElement* bin = static_cast<GstElement*>(user_data);
        // Schedule cleanup on main thread to avoid race conditions or blocking
        g_idle_add(cleanup_recording_bin, bin);
    }
    return GST_PAD_PROBE_PASS;
}

video_pipeline::video_pipeline(const std::string& name, const std::string& stream_desc, bool tee_gl_view)
    : m_name(name)
    , m_stream_desc(stream_desc)
    , m_tee_gl_view(tee_gl_view)
    , m_pipeline(nullptr)
    , m_tee(nullptr)
    , m_recording_bin(nullptr)
    , m_tee_recording_pad(nullptr)
    , m_frame_count(0)
    , m_first_buffer_recorded(false)
{
    m_preview = std::make_unique<video_preview>(name);
}

video_pipeline::~video_pipeline()
{
    stop();
    if (m_pipeline) {
        gst_object_unref(m_pipeline);
    }
}

bool video_pipeline::build()
{
    m_pipeline = gst_pipeline_new(nullptr);
    if (!m_pipeline) {
        std::cerr << "Failed to create pipeline for " << m_name << std::endl;
        return false;
    }

    GError* error = nullptr;
    // Create source bin
    GstElement* source_bin = gst_parse_bin_from_description(m_stream_desc.c_str(), TRUE, &error);
    if (!source_bin) {
        std::cerr << "Failed to create source bin for " << m_name << ": " << (error ? error->message : "unknown error") << std::endl;
        if (error) g_error_free(error);
        return false;
    }

    // Create tee and display queue
    m_tee = gst_element_factory_make("tee", "tee");
    GstElement* display_queue = gst_element_factory_make("queue", "display_queue");
    GstElement* videoconvert = gst_element_factory_make("videoconvert", "display_convert");
    GstElement* image_sink = m_preview->get_sink();

    if (!m_tee || !display_queue || !videoconvert) {
        std::cerr << "Failed to create elements for " << m_name << std::endl;
        return false;
    }

    gst_bin_add_many(GST_BIN(m_pipeline), source_bin, m_tee, display_queue, videoconvert, image_sink, nullptr);

    // Link: source -> tee
    if (!gst_element_link(source_bin, m_tee)) {
        std::cerr << "Failed to link source to tee for " << m_name << std::endl;
        return false;
    }

    // Add probe to count frames globally
    GstPad* tee_sink_pad = gst_element_get_static_pad(m_tee, "sink");
    gst_pad_add_probe(tee_sink_pad, GST_PAD_PROBE_TYPE_BUFFER, global_probe_cb, this, nullptr);
    gst_object_unref(tee_sink_pad);

    // Link: tee -> queue -> videoconvert -> sink
    // We need to request a pad from tee for the display branch
    GstPad* tee_display_pad = gst_element_request_pad_simple(m_tee, "src_%u");
    GstPad* queue_sink_pad = gst_element_get_static_pad(display_queue, "sink");
    
    if (gst_pad_link(tee_display_pad, queue_sink_pad) != GST_PAD_LINK_OK) {
        std::cerr << "Failed to link tee to display queue for " << m_name << std::endl;
        return false;
    }
    gst_object_unref(tee_display_pad);
    gst_object_unref(queue_sink_pad);

    if (!gst_element_link_many(display_queue, videoconvert, image_sink, nullptr)) {
        std::cerr << "Failed to link display branch for " << m_name << std::endl;
        return false;
    }

    // Add extra glimagesink branch if tee_gl_view is enabled
    if (m_tee_gl_view) {
        GstElement* extra_queue = gst_element_factory_make("queue", "extra_queue");
        GstElement* extra_glimagesink = gst_element_factory_make("glimagesink", "extra_glimagesink");
        
        if (!extra_queue || !extra_glimagesink) {
            std::cerr << "Failed to create extra glimagesink elements for " << m_name << std::endl;
            return false;
        }
        
        g_object_set(extra_glimagesink, "force-aspect-ratio", FALSE, nullptr);
        
        gst_bin_add_many(GST_BIN(m_pipeline), extra_queue, extra_glimagesink, nullptr);
        
        GstPad* tee_extra_pad = gst_element_request_pad_simple(m_tee, "src_%u");
        GstPad* extra_queue_sink_pad = gst_element_get_static_pad(extra_queue, "sink");
        
        if (gst_pad_link(tee_extra_pad, extra_queue_sink_pad) != GST_PAD_LINK_OK) {
            std::cerr << "Failed to link tee to extra queue for " << m_name << std::endl;
            return false;
        }
        gst_object_unref(tee_extra_pad);
        gst_object_unref(extra_queue_sink_pad);
        
        if (!gst_element_link(extra_queue, extra_glimagesink)) {
            std::cerr << "Failed to link extra queue to glimagesink for " << m_name << std::endl;
            return false;
        }
    }

    return true;
}

void video_pipeline::start()
{
    if (m_pipeline) {
        gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
    }
}

void video_pipeline::stop()
{
    if (m_recording_bin) {
        stop_recording();
    }
    if (m_pipeline) {
        gst_element_set_state(m_pipeline, GST_STATE_NULL);
    }
}

void video_pipeline::set_recording(bool enable)
{
    if (enable) {
        start_recording();
    } else {
        stop_recording();
    }
}

void video_pipeline::set_output_directory(const std::string& dir)
{
    m_output_directory = dir;
}

static std::string get_current_time_string(struct timespec* ts = nullptr) {
    struct timespec now_ts;
    clock_gettime(CLOCK_REALTIME, &now_ts);
    
    if (ts) {
        *ts = now_ts;
    }
    
    auto in_time_t = now_ts.tv_sec;
    auto ms = now_ts.tv_nsec / 1000000;
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms;
    return ss.str();
}

void video_pipeline::start_recording()
{
    if (m_recording_bin) {
        std::cout << "Already recording " << m_name << std::endl;
        return;
    }

    std::cout << "Starting recording for " << m_name << " at frame " << m_frame_count << std::endl;

    // Initialize metadata
    m_metadata = RecordingMetadata();
    m_metadata.start_frame = m_frame_count;
    m_metadata.start_abs_time = get_current_time_string(&m_metadata.start_timespec);
    m_metadata.recorded_frames = 0;
    m_metadata.average_fps = 0.0;
    m_first_buffer_recorded = false;

    // Generate filename
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    
    if (!m_output_directory.empty()) {
        std::filesystem::create_directories(m_output_directory);
        ss << m_output_directory << "/";
    }

    std::string safe_name = m_name;
    std::replace(safe_name.begin(), safe_name.end(), ' ', '_');
    
    ss << safe_name << "_" << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S") << ".mp4";
    std::string filename = ss.str();
    m_metadata.filename = filename;

    // Create elements
    m_recording_bin = gst_bin_new("recording_bin");
    GstElement* queue = gst_element_factory_make("queue", "rec_queue");
    GstElement* convert = gst_element_factory_make("videoconvert", "rec_convert");
    GstElement* enc = gst_element_factory_make("x264enc", "rec_enc");
    GstElement* mux = gst_element_factory_make("mp4mux", "rec_mux");
    GstElement* sink = gst_element_factory_make("filesink", "rec_sink");

    if (!queue || !convert || !enc || !mux || !sink) {
        std::cerr << "Failed to create recording elements" << std::endl;
        return;
    }

    g_object_set(sink, "location", filename.c_str(), nullptr);
    // Tune encoder for lower latency/CPU if needed, though defaults are usually okay
    g_object_set(enc, "tune", 0x00000004, "speed-preset", 1, nullptr); // zerolatency, ultrafast

    gst_bin_add_many(GST_BIN(m_recording_bin), queue, convert, enc, mux, sink, nullptr);
    gst_element_link_many(queue, convert, enc, mux, sink, nullptr);

    // Create ghost pad for the bin to allow linking from outside
    GstPad* queue_sink_pad = gst_element_get_static_pad(queue, "sink");
    gst_pad_add_probe(queue_sink_pad, GST_PAD_PROBE_TYPE_BUFFER, recording_probe_cb, this, nullptr);
    GstPad* ghost_pad = gst_ghost_pad_new("sink", queue_sink_pad);
    gst_element_add_pad(m_recording_bin, ghost_pad);
    gst_object_unref(queue_sink_pad);

    // Add bin to pipeline
    gst_bin_add(GST_BIN(m_pipeline), m_recording_bin);
    gst_element_sync_state_with_parent(m_recording_bin);

    // Link tee to recording bin
    m_tee_recording_pad = gst_element_request_pad_simple(m_tee, "src_%u");
    GstPad* bin_sink_pad = gst_element_get_static_pad(m_recording_bin, "sink");
    
    if (gst_pad_link(m_tee_recording_pad, bin_sink_pad) != GST_PAD_LINK_OK) {
        std::cerr << "Failed to link recording bin" << std::endl;
    }
    gst_object_unref(bin_sink_pad);
}

void video_pipeline::stop_recording()
{
    if (!m_recording_bin || !m_tee_recording_pad) {
        return;
    }

    std::cout << "Stopping recording for " << m_name << std::endl;
    
    m_metadata.stop_frame = m_frame_count;
    m_metadata.stop_abs_time = get_current_time_string(&m_metadata.stop_timespec);

    // Block the tee pad to safely unlink
    gst_pad_add_probe(m_tee_recording_pad, GST_PAD_PROBE_TYPE_BLOCK_DOWNSTREAM, unlink_cb, this, nullptr);
}

GstPadProbeReturn video_pipeline::global_probe_cb(GstPad* pad, GstPadProbeInfo* info, gpointer user_data)
{
    video_pipeline* self = static_cast<video_pipeline*>(user_data);
    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) {
        self->m_frame_count++;
    }
    return GST_PAD_PROBE_OK;
}

GstPadProbeReturn video_pipeline::recording_probe_cb(GstPad* pad, GstPadProbeInfo* info, gpointer user_data)
{
    video_pipeline* self = static_cast<video_pipeline*>(user_data);
    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) {
        GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
        if (buffer) {
            self->m_metadata.recorded_frames++;
            if (!self->m_first_buffer_recorded) {
                self->m_metadata.start_pts = GST_BUFFER_PTS(buffer);
                self->m_metadata.start_dts = GST_BUFFER_DTS(buffer);
                self->m_first_buffer_recorded = true;
            }
            self->m_metadata.stop_pts = GST_BUFFER_PTS(buffer);
            self->m_metadata.stop_dts = GST_BUFFER_DTS(buffer);
        }
    }
    return GST_PAD_PROBE_OK;
}

GstPadProbeReturn video_pipeline::unlink_cb(GstPad* pad, GstPadProbeInfo* info, gpointer user_data)
{
    video_pipeline* self = static_cast<video_pipeline*>(user_data);
    
    // Calculate FPS
    double duration_ns = (double)(self->m_metadata.stop_pts - self->m_metadata.start_pts);
    if (duration_ns > 0) {
        self->m_metadata.average_fps = (double)self->m_metadata.recorded_frames / (duration_ns / 1e9);
    } else {
        self->m_metadata.average_fps = 0.0;
    }

    std::cout << "Recording finished for " << self->m_name 
              << ": " << self->m_metadata.recorded_frames << " frames, "
              << self->m_metadata.average_fps << " FPS" << std::endl;

    // Write metadata to JSON file
    Json::Value root;
    root["filename"] = self->m_metadata.filename;
    root["start_frame"] = (Json::UInt64)self->m_metadata.start_frame;
    root["stop_frame"] = (Json::UInt64)self->m_metadata.stop_frame;
    root["start_pts"] = (Json::UInt64)self->m_metadata.start_pts;
    root["stop_pts"] = (Json::UInt64)self->m_metadata.stop_pts;
    root["start_dts"] = (Json::UInt64)self->m_metadata.start_dts;
    root["stop_dts"] = (Json::UInt64)self->m_metadata.stop_dts;
    root["start_abs_time"] = self->m_metadata.start_abs_time;
    root["stop_abs_time"] = self->m_metadata.stop_abs_time;
    root["start_timespec_sec"] = (Json::Int64)self->m_metadata.start_timespec.tv_sec;
    root["start_timespec_nsec"] = (Json::Int64)self->m_metadata.start_timespec.tv_nsec;
    root["stop_timespec_sec"] = (Json::Int64)self->m_metadata.stop_timespec.tv_sec;
    root["stop_timespec_nsec"] = (Json::Int64)self->m_metadata.stop_timespec.tv_nsec;
    root["recorded_frames"] = (Json::UInt64)self->m_metadata.recorded_frames;
    root["average_fps"] = self->m_metadata.average_fps;

    std::ofstream file(self->m_metadata.filename + ".json");
    file << root;
    file.close();

    // Remove the probe
    gst_pad_remove_probe(pad, GST_PAD_PROBE_INFO_ID(info));

    // Unlink and release pad
    GstPad* peer = gst_pad_get_peer(pad);
    if (peer) {
        gst_pad_unlink(pad, peer);
        gst_object_unref(peer);
    }
    gst_element_release_request_pad(self->m_tee, pad);
    self->m_tee_recording_pad = nullptr;

    // Send EOS to the recording bin to finalize the MP4
    GstElement* bin = self->m_recording_bin;
    self->m_recording_bin = nullptr; // Detach from object state

    // Find the filesink to attach the EOS probe
    GstElement* sink = gst_bin_get_by_name(GST_BIN(bin), "rec_sink");
    if (sink) {
        GstPad* sink_pad = gst_element_get_static_pad(sink, "sink");
        gst_pad_add_probe(sink_pad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM, eos_cb, bin, nullptr);
        gst_object_unref(sink_pad);
        gst_object_unref(sink);
    }

    gst_element_send_event(bin, gst_event_new_eos());

    return GST_PAD_PROBE_OK;
}

GtkWidget* video_pipeline::get_window() const
{
    return m_preview->get_window();
}
