// Author(s): Anton Deguet
// Copyright 2025 Johns Hopkins University

#pragma once

#include "video_preview.h"
#include <atomic>
#include <ctime>
#include <gst/gst.h>
#include <memory>
#include <string>
#include <vector>

struct video_encoding {
  int bitrate = 10000;
  int speed_preset = 2;
  int key_int_max = 60;
  int frame_rate = -1; // -1 means not specified
  int width = -1;      // -1 means not specified
  int height = -1;     // -1 means not specified
};

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
  std::vector<std::pair<int64_t, int64_t>>
      frame_timestamps; // sec, nsec for each encoded frame
};

class video_pipeline {
public:
  video_pipeline(const std::string &name, const std::string &source,
                 bool tee_gl_view = false,
                 const video_encoding &encoding = video_encoding());
  ~video_pipeline();

  bool build(void);
  void start(void);
  void stop(void);
  void set_output_directory(const std::string &dir);
  void start_recording(void);
  void stop_recording(void);
  bool is_recording() const { return m_recording_bin != nullptr; }

  GtkWidget *get_window(void) const;
  std::string get_name(void) const {
    return m_name;
  } // Getter for pipeline name

  std::string m_name;
  std::string m_source;
  std::string m_output_directory;
  bool m_tee_gl_view;
  video_encoding m_encoding;
  GstElement *m_pipeline;
  GstElement *m_tee;
  GstElement *m_recording_bin;
  GstPad *m_tee_recording_pad;
  std::unique_ptr<video_preview> m_preview;
  std::atomic<uint64_t> m_frame_count;
  RecordingMetadata m_metadata;
  bool m_first_buffer_recorded;

  static GstPadProbeReturn unlink_cb(GstPad *pad, GstPadProbeInfo *info,
                                     gpointer user_data);
  static GstPadProbeReturn global_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                           gpointer user_data);
  static GstPadProbeReturn caps_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                         gpointer user_data);
  static GstPadProbeReturn
  recording_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
  static gboolean start_recording_deferred(gpointer data);

  std::atomic<bool> m_recording_pending;
};
