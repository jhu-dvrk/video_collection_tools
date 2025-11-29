// Author(s): Anton Deguet
// Copyright 2025 Johns Hopkins University

#include "video_controller.h"
#include "gtk_stream_buf.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

video_controller::video_controller(const Json::Value &config)
    : m_config(config), m_control_window(nullptr),
      m_main_record_button(nullptr), m_dir_button(nullptr),
      m_dir_label(nullptr), m_log_view(nullptr), m_open_windows(0),
      m_quitting(false) {
  // Redirect cout
  m_old_cout_buf = std::cout.rdbuf();
  m_new_cout_buf = std::make_unique<GtkStreamBuf>();
  std::cout.rdbuf(m_new_cout_buf.get());

  if (m_config.isMember("data_directory")) {
    m_data_dir = m_config["data_directory"].asString();
  } else {
    char *cwd = g_get_current_dir();
    m_data_dir = cwd;
    g_free(cwd);
  }
}

video_controller::~video_controller() {
  // Restore cout
  if (m_old_cout_buf) {
    std::cout.rdbuf(m_old_cout_buf);
  }

  m_quitting = true;
  m_pipelines.clear();
}

void video_controller::create_ui() {
  // Create Control Window
  m_control_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(m_control_window), "Controls");
  gtk_window_set_default_size(GTK_WINDOW(m_control_window), 400, 600);
  g_signal_connect(m_control_window, "destroy",
                   G_CALLBACK(on_window_destroy_cb), this);
  m_open_windows++;

  GtkWidget *control_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
  gtk_container_set_border_width(GTK_CONTAINER(control_box), 10);
  gtk_container_add(GTK_CONTAINER(m_control_window), control_box);

  // Data Directory UI
  GtkWidget *dir_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
  gtk_box_pack_start(GTK_BOX(control_box), dir_box, FALSE, FALSE, 0);

  std::string label_text = "Data Directory: " + m_data_dir;
  m_dir_label = gtk_label_new(label_text.c_str());
  gtk_label_set_line_wrap(GTK_LABEL(m_dir_label), TRUE);
  gtk_box_pack_start(GTK_BOX(dir_box), m_dir_label, FALSE, FALSE, 0);

  m_dir_button = gtk_button_new_with_label("Change directory");
  g_signal_connect(m_dir_button, "clicked",
                   G_CALLBACK(on_dir_button_clicked_cb), this);
  gtk_box_pack_start(GTK_BOX(dir_box), m_dir_button, FALSE, FALSE, 0);

  // Main Record button
  m_main_record_button = gtk_toggle_button_new_with_label("Record");
  g_signal_connect(m_main_record_button, "toggled",
                   G_CALLBACK(on_main_record_toggled_cb), this);
  gtk_box_pack_start(GTK_BOX(control_box), m_main_record_button, FALSE, FALSE,
                     0);

  gtk_box_pack_start(GTK_BOX(control_box),
                     gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE,
                     FALSE, 5);

  const Json::Value &pipelines = m_config["pipelines"];

  for (const auto &pipeline : pipelines) {
    std::string name = pipeline.get("name", "Unknown").asString();
    std::string source = pipeline.get("source", "videotestsrc").asString();
    bool tee_gl_view = pipeline.get("tee_gl_view", false).asBool();

    // Read encoding parameters from nested "encoding" object if it exists
    video_encoding encoding;
    if (pipeline.isMember("encoding")) {
      const Json::Value &enc = pipeline["encoding"];
      encoding.bitrate = enc.get("bitrate", 10000).asInt();
      encoding.speed_preset = enc.get("speed_preset", 2).asInt();
      encoding.key_int_max = enc.get("key_int_max", 60).asInt();
      encoding.frame_rate = enc.get("frame_rate", -1).asInt();
      encoding.width = enc.get("width", -1).asInt();
      encoding.height = enc.get("height", -1).asInt();
    }

    std::cout << "Creating pipeline for: " << name << std::endl;

    auto pipe =
        std::make_unique<video_pipeline>(name, source, tee_gl_view, encoding);
    pipe->set_output_directory(m_data_dir);

    if (pipe->build()) {
      GtkWidget *window = pipe->get_window();
      g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy_cb),
                       this);

      gtk_widget_show_all(window);
      pipe->start();

      // Add control for this pipeline
      GtkWidget *row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
      GtkWidget *label = gtk_label_new(name.c_str());
      GtkWidget *check = gtk_check_button_new();

      gtk_widget_set_halign(label, GTK_ALIGN_START);
      gtk_box_pack_start(GTK_BOX(row), label, TRUE, TRUE, 0);
      gtk_box_pack_start(GTK_BOX(row), check, FALSE, FALSE, 0);

      // Set default record state for this source
      if (pipeline.isMember("record") && pipeline["record"].asBool()) {
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check), TRUE);
      }

      g_signal_connect(check, "toggled", G_CALLBACK(on_source_check_toggled_cb),
                       this);
      m_checkboxes.push_back(check);

      gtk_box_pack_start(GTK_BOX(control_box), row, FALSE, FALSE, 0);

      m_pipelines.push_back(std::move(pipe));
      m_open_windows++;
    }
  }

  gtk_box_pack_start(GTK_BOX(control_box),
                     gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE,
                     FALSE, 5);

  // Log Window
  GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
  gtk_widget_set_size_request(scrolled_window, -1, 150);
  gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
                                 GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);

  m_log_view = gtk_text_view_new();
  gtk_text_view_set_editable(GTK_TEXT_VIEW(m_log_view), FALSE);
  gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(m_log_view), GTK_WRAP_WORD);
  gtk_container_add(GTK_CONTAINER(scrolled_window), m_log_view);

  gtk_box_pack_start(GTK_BOX(control_box), scrolled_window, TRUE, TRUE, 5);

  // Connect log view to stream buffer
  static_cast<GtkStreamBuf *>(m_new_cout_buf.get())
      ->set_widgets(GTK_TEXT_VIEW(m_log_view));

  // Quit button
  GtkWidget *quit_btn = gtk_button_new_with_label("Quit");
  g_signal_connect(quit_btn, "clicked", G_CALLBACK(on_quit_clicked_cb), this);
  gtk_box_pack_end(GTK_BOX(control_box), quit_btn, FALSE, FALSE, 0);

  // Ensure initial recording state is correct based on all buttons
  update_recording_state();

  gtk_widget_show_all(m_control_window);
}

bool video_controller::has_open_windows() const { return m_open_windows > 0; }

void video_controller::update_recording_state() {
  if (!m_main_record_button)
    return;

  gboolean recording_active =
      gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_main_record_button));

  for (size_t i = 0; i < m_pipelines.size(); ++i) {
    if (i < m_checkboxes.size()) {
      gboolean selected =
          gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_checkboxes[i]));
      if (recording_active && selected) {
        m_pipelines[i]->start_recording();
      } else {
        m_pipelines[i]->stop_recording();
      }
    }
  }
}

void video_controller::on_window_destroy() {
  if (m_quitting)
    return;
  m_open_windows--;
  if (m_open_windows <= 0) {
    m_quitting = true;
    // Stop any active recordings and wait briefly for them to finish
    for (auto &pipe : m_pipelines) {
      if (pipe && pipe->is_recording()) {
        pipe->stop_recording();
      }
    }

    // Wait up to 60 seconds for recordings to finish (log progress)
    const auto start_wait = std::chrono::steady_clock::now();
    const auto deadline = start_wait + std::chrono::seconds(60);
    int last_remaining = -1;
    std::cout << "Waiting up to 60s for recordings to finish..." << std::endl;
    while (std::chrono::steady_clock::now() < deadline) {
      bool any_recording = false;
      int recording_count = 0;
      for (auto &pipe : m_pipelines) {
        if (pipe && pipe->is_recording()) {
          any_recording = true;
          ++recording_count;
        }
      }
      if (!any_recording)
        break;

      auto now = std::chrono::steady_clock::now();
      int remaining = static_cast<int>(
          std::chrono::duration_cast<std::chrono::seconds>(deadline - now)
              .count());
      if (remaining != last_remaining) {
        std::cout << "Waiting: " << remaining << "s remaining, "
                  << recording_count << " pipeline(s) still recording..."
                  << std::endl;
        last_remaining = remaining;
      }

      // Process GTK events so GStreamer and UI can progress
      while (gtk_events_pending())
        gtk_main_iteration_do(FALSE);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    m_pipelines.clear();
    gtk_main_quit();
  }
}

void video_controller::on_main_record_toggled() { update_recording_state(); }

void video_controller::on_source_check_toggled() { update_recording_state(); }

void video_controller::on_quit_clicked() {
  m_quitting = true;
  // Stop any active recordings first
  for (auto &pipe : m_pipelines) {
    if (pipe && pipe->is_recording()) {
      pipe->stop_recording();
    }
  }

  // Wait up to 60 seconds for recordings to finish (log progress)
  const auto start_wait = std::chrono::steady_clock::now();
  const auto deadline = start_wait + std::chrono::seconds(60);
  int last_remaining = -1;
  std::cout << "Waiting up to 60s for recordings to finish..." << std::endl;
  while (std::chrono::steady_clock::now() < deadline) {
    bool any_recording = false;
    int recording_count = 0;
    for (auto &pipe : m_pipelines) {
      if (pipe && pipe->is_recording()) {
        any_recording = true;
        ++recording_count;
      }
    }
    if (!any_recording)
      break;

    auto now = std::chrono::steady_clock::now();
    int remaining = static_cast<int>(
        std::chrono::duration_cast<std::chrono::seconds>(deadline - now)
            .count());
    if (remaining != last_remaining) {
      std::cout << "Waiting: " << remaining << "s remaining, "
                << recording_count << " pipeline(s) still recording..."
                << std::endl;
      last_remaining = remaining;
    }

    while (gtk_events_pending())
      gtk_main_iteration_do(FALSE);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  m_pipelines.clear();
  gtk_main_quit();
}

void video_controller::on_dir_button_clicked() {
  GtkWidget *dialog;
  GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER;
  gint res;

  dialog = gtk_file_chooser_dialog_new(
      "Select Data Directory", GTK_WINDOW(m_control_window), action, "_Cancel",
      GTK_RESPONSE_CANCEL, "_Open", GTK_RESPONSE_ACCEPT, NULL);

  if (!m_data_dir.empty()) {
    gtk_file_chooser_set_current_folder(GTK_FILE_CHOOSER(dialog),
                                        m_data_dir.c_str());
  }

  res = gtk_dialog_run(GTK_DIALOG(dialog));
  if (res == GTK_RESPONSE_ACCEPT) {
    char *filename;
    GtkFileChooser *chooser = GTK_FILE_CHOOSER(dialog);
    filename = gtk_file_chooser_get_filename(chooser);

    m_data_dir = filename;
    g_free(filename);

    std::cout << "Selected data directory: " << m_data_dir << std::endl;

    std::string label_text = "Data Directory: " + m_data_dir;
    gtk_label_set_text(GTK_LABEL(m_dir_label), label_text.c_str());

    // Check if currently recording
    gboolean was_recording =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_main_record_button));

    if (was_recording) {
      // Stop recording
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(m_main_record_button),
                                   FALSE);

      // Process pending events to ensure UI updates and stop commands are sent
      while (gtk_events_pending())
        gtk_main_iteration();
    }

    // Update all pipelines with the new directory
    for (auto &pipe : m_pipelines) {
      pipe->set_output_directory(m_data_dir);
    }

    if (was_recording) {
      // Restart recording after a short delay to allow GStreamer to cleanup
      g_timeout_add(500, restart_recording_cb, this);
    }
  }

  gtk_widget_destroy(dialog);
}

// Static callbacks
gboolean video_controller::restart_recording_cb(gpointer data) {
  video_controller *self = static_cast<video_controller *>(data);
  if (self->m_main_record_button) {
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(self->m_main_record_button),
                                 TRUE);
  }
  return G_SOURCE_REMOVE;
}

// ROS2 integration: set recording by pipeline name
void video_controller::set_recording_by_name(const std::string &name,
                                             bool enable) {
  if (name.empty()) {
    // If name is empty, apply to all pipelines
    for (size_t i = 0; i < m_pipelines.size(); ++i) {
      if (m_pipelines[i]) {
        // ...existing code...
        if (i < m_checkboxes.size()) {
          gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(m_checkboxes[i]),
                                       enable ? TRUE : FALSE);
        }
      }
    }
    std::cout << (enable ? "Recording enabled for all pipelines"
                         : "Recording disabled for all pipelines")
              << std::endl;
  } else {
    for (size_t i = 0; i < m_pipelines.size(); ++i) {
      if (m_pipelines[i] && m_pipelines[i]->get_name() == name) {
        // ...existing code...
        std::cout << (enable ? "Recording enabled for pipeline: "
                             : "Recording disabled for pipeline: ")
                  << name << std::endl;
        // Update GUI checkbox if available
        if (i < m_checkboxes.size()) {
          gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(m_checkboxes[i]),
                                       enable ? TRUE : FALSE);
        }
      }
    }
  }
}

// ROS2 integration: set data directory
void video_controller::set_data_directory(const std::string &dir) {
  m_data_dir = dir;
  std::cout << "Set data directory: " << m_data_dir << std::endl;
  std::string label_text = "Data Directory: " + m_data_dir;
  if (m_dir_label) {
    gtk_label_set_text(GTK_LABEL(m_dir_label), label_text.c_str());
  }
  for (auto &pipe : m_pipelines) {
    pipe->set_output_directory(m_data_dir);
  }
}

void video_controller::on_window_destroy_cb(GtkWidget *widget, gpointer data) {
  video_controller *self = static_cast<video_controller *>(data);
  if (widget == self->m_control_window) {
    self->on_quit_clicked();
  } else {
    self->on_window_destroy();
  }
}

void video_controller::on_main_record_toggled_cb(GtkToggleButton *button,
                                                 gpointer data) {
  static_cast<video_controller *>(data)->on_main_record_toggled();
}

void video_controller::on_source_check_toggled_cb(GtkToggleButton *button,
                                                  gpointer data) {
  static_cast<video_controller *>(data)->on_source_check_toggled();
}

void video_controller::on_quit_clicked_cb(GtkButton *button, gpointer data) {
  static_cast<video_controller *>(data)->on_quit_clicked();
}

void video_controller::on_dir_button_clicked_cb(GtkButton *button,
                                                gpointer data) {
  static_cast<video_controller *>(data)->on_dir_button_clicked();
}
