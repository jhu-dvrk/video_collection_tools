// Author(s): Anton Deguet
// Copyright 2025 Johns Hopkins University

#include "video_preview.h"
#include <gdk/gdkx.h>
#include <iostream>
#include <stdexcept>

video_preview::video_preview(const std::string &name)
    : m_name(name), m_sink(nullptr) {
  m_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(m_window), name.c_str());
  gtk_window_set_default_size(GTK_WINDOW(m_window), 400, 300);

  m_video_area = gtk_drawing_area_new();
  gtk_widget_set_hexpand(m_video_area, TRUE);
  gtk_widget_set_vexpand(m_video_area, TRUE);

  gtk_container_add(GTK_CONTAINER(m_window), m_video_area);

  gtk_widget_add_events(m_video_area,
                        GDK_BUTTON_PRESS_MASK | GDK_STRUCTURE_MASK);

  g_signal_connect(m_video_area, "button-press-event",
                   G_CALLBACK(on_button_press), this);
  g_signal_connect(m_video_area, "size-allocate", G_CALLBACK(on_size_allocate),
                   this);
  g_signal_connect(m_video_area, "realize", G_CALLBACK(on_realize), this);

  m_sink = gst_element_factory_make("glimagesink", "sink");
  if (!m_sink) {
    throw std::runtime_error("Failed to create glimagesink");
  }

  // Configure sink for free resizing initially (or default)
  g_object_set(m_sink, "force-aspect-ratio", FALSE, "handle-events", FALSE,
               nullptr);
}

video_preview::~video_preview() {
  if (m_window) {
    gtk_widget_destroy(m_window);
  }
}

GtkWidget *video_preview::get_window() const { return m_window; }

GstElement *video_preview::get_sink() const { return m_sink; }

void video_preview::on_realize(GtkWidget *widget, gpointer data) {
  video_preview *self = static_cast<video_preview *>(data);
  self->handle_realize();
}

void video_preview::handle_realize() {
  // Set video overlay window handle using X11
  GdkWindow *gdk_window = gtk_widget_get_window(m_video_area);
  if (gdk_window && m_sink) {
    guintptr window_handle = GDK_WINDOW_XID(gdk_window);
    GstVideoOverlay *overlay = GST_VIDEO_OVERLAY(m_sink);
    gst_video_overlay_set_window_handle(overlay, window_handle);
  }
}

void video_preview::on_size_allocate(GtkWidget *widget,
                                     GtkAllocation *allocation, gpointer data) {
  video_preview *self = static_cast<video_preview *>(data);
  self->handle_size_allocate(allocation);
}

void video_preview::handle_size_allocate(GtkAllocation *allocation) {
  if (m_sink) {
    GstVideoOverlay *overlay = GST_VIDEO_OVERLAY(m_sink);
    gst_video_overlay_set_render_rectangle(overlay, allocation->x,
                                           allocation->y, allocation->width,
                                           allocation->height);
  }
}

gboolean video_preview::on_button_press(GtkWidget *widget,
                                        GdkEventButton *event, gpointer data) {
  video_preview *self = static_cast<video_preview *>(data);
  return self->handle_button_press(event);
}

gboolean video_preview::handle_button_press(GdkEventButton *event) {
  if (event->type == GDK_BUTTON_PRESS && event->button == 3) { // Right click
    GtkWidget *menu = gtk_menu_new();

    GtkWidget *restore_item =
        gtk_menu_item_new_with_label("Restore Aspect Ratio");
    g_signal_connect(restore_item, "activate",
                     G_CALLBACK(on_restore_aspect_ratio), this);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu), restore_item);

    GtkWidget *free_item = gtk_menu_item_new_with_label("Free Aspect Ratio");
    g_signal_connect(free_item, "activate", G_CALLBACK(on_free_aspect_ratio),
                     this);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu), free_item);

    gtk_widget_show_all(menu);
    gtk_menu_popup_at_pointer(GTK_MENU(menu), (GdkEvent *)event);

    return TRUE;
  }
  return FALSE;
}

void video_preview::on_restore_aspect_ratio(GtkMenuItem *menu_item,
                                            gpointer data) {
  video_preview *self = static_cast<video_preview *>(data);
  self->restore_aspect_ratio();
}

void video_preview::restore_aspect_ratio() {
  if (m_sink && m_window) {
    g_object_set(m_sink, "force-aspect-ratio", TRUE, nullptr);

    // Get video dimensions from negotiated caps
    GstPad *sink_pad = gst_element_get_static_pad(m_sink, "sink");
    if (sink_pad) {
      GstCaps *caps = gst_pad_get_current_caps(sink_pad);
      if (caps) {
        GstStructure *structure = gst_caps_get_structure(caps, 0);
        gint vid_width = 0, vid_height = 0;

        if (gst_structure_get_int(structure, "width", &vid_width) &&
            gst_structure_get_int(structure, "height", &vid_height)) {

          if (vid_width > 0 && vid_height > 0) {
            gint win_width, win_height;
            gtk_window_get_size(GTK_WINDOW(m_window), &win_width, &win_height);

            double vid_aspect = (double)vid_width / (double)vid_height;
            double win_aspect = (double)win_width / (double)win_height;

            if (win_aspect > vid_aspect) {
              // Window is too wide, shrink width
              win_width = (gint)(win_height * vid_aspect);
            } else {
              // Window is too tall, shrink height
              win_height = (gint)(win_width / vid_aspect);
            }

            gtk_window_resize(GTK_WINDOW(m_window), win_width, win_height);
          }
        }

        gst_caps_unref(caps);
      }
      gst_object_unref(sink_pad);
    }
  }
}

void video_preview::on_free_aspect_ratio(GtkMenuItem *menu_item,
                                         gpointer data) {
  video_preview *self = static_cast<video_preview *>(data);
  self->free_aspect_ratio();
}

void video_preview::free_aspect_ratio() {
  if (m_sink && m_video_area) {
    g_object_set(m_sink, "force-aspect-ratio", FALSE, nullptr);

    GtkAllocation allocation;
    gtk_widget_get_allocation(m_video_area, &allocation);

    handle_size_allocate(&allocation);

    GstVideoOverlay *overlay = GST_VIDEO_OVERLAY(m_sink);
    gst_video_overlay_expose(overlay);
  }
}
