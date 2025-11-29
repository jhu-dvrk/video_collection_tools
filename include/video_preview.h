// Author(s): Anton Deguet
// Copyright 2025 Johns Hopkins University

#pragma once

#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <gtk/gtk.h>
#include <string>

class video_preview {
public:
  video_preview(const std::string &name);
  ~video_preview();

  GtkWidget *get_window() const;
  GstElement *get_sink() const;
  void set_title(const std::string &title);

private:
  std::string m_name;
  GtkWidget *m_window;
  GtkWidget *m_video_area;
  GstElement *m_sink;
  bool m_sink_embedded;

  static void on_realize(GtkWidget *widget, gpointer data);
  static void on_size_allocate(GtkWidget *widget, GtkAllocation *allocation,
                               gpointer data);
  static gboolean on_button_press(GtkWidget *widget, GdkEventButton *event,
                                  gpointer data);

  static void on_restore_aspect_ratio(GtkMenuItem *menu_item, gpointer data);
  static void on_free_aspect_ratio(GtkMenuItem *menu_item, gpointer data);

  void handle_realize();
  void handle_size_allocate(GtkAllocation *allocation);
  gboolean handle_button_press(GdkEventButton *event);
  void restore_aspect_ratio();
  void free_aspect_ratio();
};
