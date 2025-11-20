#pragma once

#include <gst/gst.h>
#include <string>
#include <memory>
#include <atomic>
#include <ctime>
#include "video_preview.h"

struct RecordingMetadata {
    std::string filename;
    uint64_t start_frame;
    uint64_t stop_frame;
    uint64_t start_pts;
    uint64_t stop_pts;
    uint64_t start_dts;
    uint64_t stop_dts;
    std::string start_abs_time;
    std::string stop_abs_time;
    struct timespec start_timespec;
    struct timespec stop_timespec;
    uint64_t recorded_frames;
    double average_fps;
};

class video_pipeline {
public:
    video_pipeline(const std::string& name, const std::string& stream_desc, bool tee_gl_view = false);
    ~video_pipeline();

    bool build();
    void start();
    void stop();
    void set_recording(bool enable);
    void set_output_directory(const std::string& dir);
    
    GtkWidget* get_window() const;

private:
    std::string m_name;
    std::string m_stream_desc;
    std::string m_output_directory;
    bool m_tee_gl_view;
    GstElement* m_pipeline;
    GstElement* m_tee;
    GstElement* m_recording_bin;
    GstPad* m_tee_recording_pad;
    std::unique_ptr<video_preview> m_preview;
    std::atomic<uint64_t> m_frame_count;
    RecordingMetadata m_metadata;
    bool m_first_buffer_recorded;

    void start_recording();
    void stop_recording();
    static GstPadProbeReturn unlink_cb(GstPad* pad, GstPadProbeInfo* info, gpointer user_data);
    static GstPadProbeReturn global_probe_cb(GstPad* pad, GstPadProbeInfo* info, gpointer user_data);
    static GstPadProbeReturn recording_probe_cb(GstPad* pad, GstPadProbeInfo* info, gpointer user_data);
};
