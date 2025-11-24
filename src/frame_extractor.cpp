// Author(s): Anton Deguet
// Copyright 2025 Johns Hopkins University

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <json/json.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <vector>

struct FrameInfo {
    uint64_t frame_number;
    uint64_t pts;
    uint64_t dts;
    std::string filename;
    std::string timestamp;
    struct timespec timespec_ts;
    int64_t abs_sec = 0;
    int64_t abs_nsec = 0;
};

class FrameExtractor {
public:
    FrameExtractor(const std::string& video_file, const std::string& json_file)
        : m_video_file(video_file)
        , m_json_file(json_file)
        , m_pipeline(nullptr)
        , m_frame_count(0)
        , m_start_time_ms(0)
        , m_fps(30.0)
        , m_total_frames(0)
    {
        // Parse JSON metadata
        std::ifstream file(json_file);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open JSON file: " + json_file);
        }
        Json::Reader reader;
        if (!reader.parse(file, m_metadata)) {
            throw std::runtime_error("Failed to parse JSON: " + reader.getFormattedErrorMessages());
        }

        // Load frame timestamps
        if (m_metadata.isMember("frame_timestamps")) {
            const Json::Value& ts_array = m_metadata["frame_timestamps"];
            for (const auto& entry : ts_array) {
                m_frame_timestamps.push_back({entry["sec"].asInt64(), entry["nsec"].asInt64()});
            }
        }

        // Parse start time
        if (m_metadata.isMember("start_abs_time")) {
            m_start_abs_time = m_metadata["start_abs_time"].asString();
            m_start_time_ms = parse_timestamp(m_start_abs_time);
        }

        // Load start_timespec from metadata
        if (m_metadata.isMember("start_timespec_sec") && m_metadata.isMember("start_timespec_nsec")) {
            m_start_timespec.tv_sec = m_metadata["start_timespec_sec"].asInt64();
            m_start_timespec.tv_nsec = m_metadata["start_timespec_nsec"].asInt64();
        } else {
            // Fallback to converting from start_time_ms if timespec not available
            m_start_timespec.tv_sec = m_start_time_ms / 1000;
            m_start_timespec.tv_nsec = (m_start_time_ms % 1000) * 1000000;
        }

        // Get FPS
        if (m_metadata.isMember("average_fps")) {
            m_fps = m_metadata["average_fps"].asDouble();
        }

        // Create output directory
        std::filesystem::path video_path(video_file);
        m_output_dir = video_path.stem().string() + "_frames";
        std::filesystem::create_directories(m_output_dir);

        std::cout << "Output directory: " << m_output_dir << std::endl;
    }
    
    ~FrameExtractor() {
        if (m_pipeline) {
            gst_element_set_state(m_pipeline, GST_STATE_NULL);
            gst_object_unref(m_pipeline);
        }
    }
    
    bool run() {
        // Build pipeline: filesrc ! decodebin ! videoconvert ! pngenc ! appsink
        std::stringstream pipeline_desc;
        pipeline_desc << "filesrc location=\"" << m_video_file << "\" ! "
                      << "decodebin ! videoconvert ! pngenc ! appsink name=sink";
        
        GError* error = nullptr;
        m_pipeline = gst_parse_launch(pipeline_desc.str().c_str(), &error);
        
        if (!m_pipeline || error) {
            std::cerr << "Failed to create pipeline: " 
                      << (error ? error->message : "unknown error") << std::endl;
            if (error) g_error_free(error);
            return false;
        }
        
        // Get appsink
        GstElement* appsink = gst_bin_get_by_name(GST_BIN(m_pipeline), "sink");
        if (!appsink) {
            std::cerr << "Failed to get appsink" << std::endl;
            return false;
        }
        
        // Configure appsink
        g_object_set(appsink, "emit-signals", TRUE, "sync", FALSE, nullptr);
        g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample_cb), this);
        
        gst_object_unref(appsink);
        
        // Start pipeline
        GstStateChangeReturn ret = gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cerr << "Failed to start pipeline" << std::endl;
            return false;
        }
        
        // Wait for pipeline to be ready and query duration
        gst_element_get_state(m_pipeline, nullptr, nullptr, GST_CLOCK_TIME_NONE);
        
        gint64 duration_ns = 0;
        if (gst_element_query_duration(m_pipeline, GST_FORMAT_TIME, &duration_ns)) {
            double duration_sec = duration_ns / 1e9;
            m_total_frames = static_cast<uint64_t>(duration_sec * m_fps);
            std::cout << "Estimated total frames: " << m_total_frames << std::endl;
        }
        
        // Wait for EOS or error
        GstBus* bus = gst_element_get_bus(m_pipeline);
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, GST_CLOCK_TIME_NONE,
            (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        
        if (msg) {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
                GError* err;
                gchar* debug_info;
                gst_message_parse_error(msg, &err, &debug_info);
                std::cerr << "Error: " << err->message << std::endl;
                if (debug_info) {
                    std::cerr << "Debug: " << debug_info << std::endl;
                    g_free(debug_info);
                }
                g_error_free(err);
                gst_message_unref(msg);
                gst_object_unref(bus);
                return false;
            }
            gst_message_unref(msg);
        }
        
        gst_object_unref(bus);
        
        // Write index file
        write_index();
        
        std::cout << std::endl; // Move to next line after progress updates
        std::cout << "Extracted " << m_frame_count << " frames" << std::endl;
        return true;
    }

private:
    static GstFlowReturn on_new_sample_cb(GstElement* sink, gpointer user_data) {
        FrameExtractor* self = static_cast<FrameExtractor*>(user_data);
        return self->on_new_sample(sink);
    }
    
    uint64_t parse_timestamp(const std::string& timestamp) {
        // Parse format: "2025-11-19 14:30:00.123"
        std::tm tm = {};
        int milliseconds = 0;
        
        std::istringstream ss(timestamp);
        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
        
        // Extract milliseconds
        size_t dot_pos = timestamp.find('.');
        if (dot_pos != std::string::npos && dot_pos + 1 < timestamp.length()) {
            std::string ms_str = timestamp.substr(dot_pos + 1);
            milliseconds = std::stoi(ms_str);
        }
        
        auto time_point = std::chrono::system_clock::from_time_t(std::mktime(&tm));
        auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
            time_point.time_since_epoch()).count();
        
        return ms_since_epoch + milliseconds;
    }
    
    std::string format_timestamp(uint64_t ms_since_epoch, struct timespec* ts = nullptr) {
        time_t seconds = ms_since_epoch / 1000;
        long nanoseconds = (ms_since_epoch % 1000) * 1000000;
        
        if (ts) {
            ts->tv_sec = seconds;
            ts->tv_nsec = nanoseconds;
        }
        
        auto ms = ms_since_epoch % 1000;
        std::stringstream ss;
        ss << std::put_time(std::localtime(&seconds), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms;
        return ss.str();
    }
    
    std::string format_timestamp_from_timespec(const struct timespec& ts) {
        auto ms = ts.tv_nsec / 1000000;
        std::stringstream ss;
        ss << std::put_time(std::localtime(&ts.tv_sec), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms;
        return ss.str();
    }
    
    GstFlowReturn on_new_sample(GstElement* sink) {
        GstSample* sample = nullptr;
        g_signal_emit_by_name(sink, "pull-sample", &sample);

        if (!sample) {
            return GST_FLOW_ERROR;
        }

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        // Get buffer metadata
        GstClockTime pts = GST_BUFFER_PTS(buffer);
        GstClockTime dts = GST_BUFFER_DTS(buffer);

        // Generate filename
        std::stringstream filename;
        filename << "frame_" << std::setfill('0') << std::setw(6) << m_frame_count << ".png";

        std::string filepath = m_output_dir + "/" + filename.str();

        // Write PNG data to file
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            std::ofstream out(filepath, std::ios::binary);
            out.write(reinterpret_cast<const char*>(map.data), map.size);
            out.close();
            gst_buffer_unmap(buffer, &map);
        }

        // Use saved absolute timestamp if available
        struct timespec frame_ts = {0, 0};
        int64_t abs_sec = 0, abs_nsec = 0;
        if (m_frame_count < m_frame_timestamps.size()) {
            abs_sec = m_frame_timestamps[m_frame_count].first;
            abs_nsec = m_frame_timestamps[m_frame_count].second;
            frame_ts.tv_sec = abs_sec;
            frame_ts.tv_nsec = abs_nsec;
        } else {
            // Fallback to previous method if not available
            double frame_offset_sec = m_frame_count / m_fps;
            long frame_offset_nsec = static_cast<long>((frame_offset_sec - static_cast<long>(frame_offset_sec)) * 1e9);
            long frame_offset_sec_int = static_cast<long>(frame_offset_sec);
            frame_ts.tv_sec = m_start_timespec.tv_sec + frame_offset_sec_int;
            frame_ts.tv_nsec = m_start_timespec.tv_nsec + frame_offset_nsec;
            if (frame_ts.tv_nsec >= 1000000000) {
                frame_ts.tv_sec += frame_ts.tv_nsec / 1000000000;
                frame_ts.tv_nsec = frame_ts.tv_nsec % 1000000000;
            }
        }

        // Store frame info
        FrameInfo info;
        info.frame_number = m_frame_count;
        info.pts = pts;
        info.dts = dts;
        info.filename = filename.str();
        info.timespec_ts = frame_ts;
        info.timestamp = format_timestamp_from_timespec(frame_ts);
        info.abs_sec = abs_sec;
        info.abs_nsec = abs_nsec;
        m_frames.push_back(info);

        m_frame_count++;

        if (m_total_frames > 0) {
            double progress = (m_frame_count * 100.0) / m_total_frames;
            std::cout << "\rProgress: " << std::fixed << std::setprecision(1)
                        << progress << "% (" << m_frame_count << "/" << m_total_frames << ")"
                        << std::flush;
        } else {
            std::cout << "\rExtracted " << m_frame_count << " frames..." << std::flush;
        }

        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    
    void write_index() {
        std::string index_file = m_output_dir + "/index.json";
        
        Json::Value root;
        root["video_file"] = m_video_file;
        root["total_frames"] = (Json::UInt64)m_frame_count;
        
        // Copy metadata from original JSON
        if (m_metadata.isMember("start_abs_time")) {
            root["start_abs_time"] = m_metadata["start_abs_time"];
        }
        if (m_metadata.isMember("stop_abs_time")) {
            root["stop_abs_time"] = m_metadata["stop_abs_time"];
        }
        if (m_metadata.isMember("average_fps")) {
            root["average_fps"] = m_metadata["average_fps"];
        }
        
        Json::Value frames(Json::arrayValue);
        for (const auto& frame : m_frames) {
            Json::Value frame_obj;
            frame_obj["frame_number"] = (Json::UInt64)frame.frame_number;
            frame_obj["timestamp"] = frame.timestamp;
            frame_obj["timespec_sec"] = (Json::Int64)frame.timespec_ts.tv_sec;
            frame_obj["timespec_nsec"] = (Json::Int64)frame.timespec_ts.tv_nsec;
            frame_obj["pts"] = (Json::UInt64)frame.pts;
            frame_obj["dts"] = (Json::UInt64)frame.dts;
            frame_obj["filename"] = frame.filename;
            frames.append(frame_obj);
        }
        root["frames"] = frames;
        
        std::ofstream out(index_file);
        out << root;
        out.close();
        
        std::cout << "Index file written to: " << index_file << std::endl;
    }
    
    std::string m_video_file;
    std::string m_json_file;
    std::string m_output_dir;
    GstElement* m_pipeline;
    uint64_t m_frame_count;
    Json::Value m_metadata;
    std::vector<FrameInfo> m_frames;
    std::vector<std::pair<int64_t, int64_t>> m_frame_timestamps;
    std::string m_start_abs_time;
    uint64_t m_start_time_ms;
    struct timespec m_start_timespec;
    double m_fps;
    uint64_t m_total_frames;
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <video.mp4>" << std::endl;
        return 1;
    }
    
    std::string video_file = argv[1];
    std::string json_file = video_file + ".json";
    
    // Check if files exist
    if (!std::filesystem::exists(video_file)) {
        std::cerr << "Video file not found: " << video_file << std::endl;
        return 1;
    }
    
    if (!std::filesystem::exists(json_file)) {
        std::cerr << "JSON file not found: " << json_file << std::endl;
        return 1;
    }
    
    // Initialize GStreamer
    gst_init(&argc, &argv);
    
    try {
        FrameExtractor extractor(video_file, json_file);
        if (!extractor.run()) {
            std::cerr << "Frame extraction failed" << std::endl;
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
