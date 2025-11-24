// Author(s): Anton Deguet
// Copyright 2025 Johns Hopkins University

#pragma once
#include "video_pipeline.h"
#include <gtk/gtk.h>
#include <json/json.h>
#include <memory>
#include <vector>

class video_controller {
public:
  video_controller(const Json::Value &config);
  ~video_controller();

  void create_ui();
  bool has_open_windows() const;

  // ROS2 integration: set recording by pipeline name
  void set_recording_by_name(const std::string &name, bool enable);
  // ROS2 integration: unset recording by pipeline name
  void unset_recording_by_name(const std::string &name);
  // ROS2 integration: set data directory
  void set_data_directory(const std::string &dir);

private:
  Json::Value m_config;
  GtkWidget *m_control_window;
  GtkWidget *m_main_record_button;
  GtkWidget *m_dir_button;
  GtkWidget *m_dir_label;
  GtkWidget *m_log_view;
  std::string m_data_dir;

  std::vector<std::unique_ptr<video_pipeline>> m_pipelines;
  std::vector<GtkWidget *> m_checkboxes;

  int m_open_windows;
  bool m_quitting;

  // Cout redirection
  std::streambuf *m_old_cout_buf;
  std::unique_ptr<std::streambuf> m_new_cout_buf;

  void update_recording_state();
  void on_window_destroy();
  void on_main_record_toggled();
  void on_source_check_toggled();
  void on_quit_clicked();
  void on_dir_button_clicked();

  // Static callbacks
  static void on_window_destroy_cb(GtkWidget *widget, gpointer data);
  static void on_main_record_toggled_cb(GtkToggleButton *button, gpointer data);
  static void on_source_check_toggled_cb(GtkToggleButton *button,
                                         gpointer data);
  static void on_quit_clicked_cb(GtkButton *button, gpointer data);
  static void on_dir_button_clicked_cb(GtkButton *button, gpointer data);
  static gboolean restart_recording_cb(gpointer data);
};
