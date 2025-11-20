#include "video_controller.h"
#include <iostream>

video_controller::video_controller(const Json::Value& config)
    : m_config(config)
    , m_control_window(nullptr)
    , m_main_record_button(nullptr)
    , m_dir_button(nullptr)
    , m_dir_label(nullptr)
    , m_open_windows(0)
    , m_quitting(false)
{
    if (m_config.isMember("data_directory")) {
        m_data_dir = m_config["data_directory"].asString();
    } else {
        char* cwd = g_get_current_dir();
        m_data_dir = cwd;
        g_free(cwd);
    }
}

video_controller::~video_controller()
{
    m_quitting = true;
    m_pipelines.clear();
}

void video_controller::create_ui()
{
    // Create Control Window
    m_control_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(m_control_window), "Controls");
    gtk_window_set_default_size(GTK_WINDOW(m_control_window), 300, 400);
    g_signal_connect(m_control_window, "destroy", G_CALLBACK(on_window_destroy_cb), this);
    m_open_windows++;

    GtkWidget* control_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_set_border_width(GTK_CONTAINER(control_box), 10);
    gtk_container_add(GTK_CONTAINER(m_control_window), control_box);

    // Data Directory UI
    GtkWidget* dir_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    gtk_box_pack_start(GTK_BOX(control_box), dir_box, FALSE, FALSE, 0);

    std::string label_text = "Data Directory: " + m_data_dir;
    m_dir_label = gtk_label_new(label_text.c_str());
    gtk_label_set_line_wrap(GTK_LABEL(m_dir_label), TRUE);
    gtk_box_pack_start(GTK_BOX(dir_box), m_dir_label, FALSE, FALSE, 0);

    m_dir_button = gtk_button_new_with_label("Change");
    g_signal_connect(m_dir_button, "clicked", G_CALLBACK(on_dir_button_clicked_cb), this);
    gtk_box_pack_start(GTK_BOX(dir_box), m_dir_button, FALSE, FALSE, 0);

    // Main Record button
    m_main_record_button = gtk_toggle_button_new_with_label("Record");
    g_signal_connect(m_main_record_button, "toggled", G_CALLBACK(on_main_record_toggled_cb), this);
    gtk_box_pack_start(GTK_BOX(control_box), m_main_record_button, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(control_box), gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE, 5);

    const Json::Value& sources = m_config["sources"];

    for (const auto& source : sources) {
        std::string name = source.get("name", "Unknown").asString();
        std::string stream = source.get("stream", "videotestsrc").asString();
        
        std::cout << "Creating pipeline for: " << name << std::endl;

        auto pipe = std::make_unique<video_pipeline>(name, stream);
        pipe->set_output_directory(m_data_dir);
        
        if (pipe->build()) {
            GtkWidget* window = pipe->get_window();
            g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy_cb), this);
            
            gtk_widget_show_all(window);
            pipe->start();
            
            // Add control for this pipeline
            GtkWidget* row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
            GtkWidget* label = gtk_label_new(name.c_str());
            GtkWidget* check = gtk_check_button_new();
            
            gtk_widget_set_halign(label, GTK_ALIGN_START);
            gtk_box_pack_start(GTK_BOX(row), label, TRUE, TRUE, 0);
            gtk_box_pack_start(GTK_BOX(row), check, FALSE, FALSE, 0);
            
            // Set default record state for this source
            if (source.isMember("record") && source["record"].asBool()) {
                gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check), TRUE);
            }

            g_signal_connect(check, "toggled", G_CALLBACK(on_source_check_toggled_cb), this);
            m_checkboxes.push_back(check);
            
            gtk_box_pack_start(GTK_BOX(control_box), row, FALSE, FALSE, 0);

            m_pipelines.push_back(std::move(pipe));
            m_open_windows++;
        }
    }

    gtk_box_pack_start(GTK_BOX(control_box), gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE, 5);

    // Quit button
    GtkWidget* quit_btn = gtk_button_new_with_label("Quit");
    g_signal_connect(quit_btn, "clicked", G_CALLBACK(on_quit_clicked_cb), this);
    gtk_box_pack_end(GTK_BOX(control_box), quit_btn, FALSE, FALSE, 0);

    // Ensure initial recording state is correct based on all buttons
    update_recording_state();

    gtk_widget_show_all(m_control_window);
}

bool video_controller::has_open_windows() const
{
    return m_open_windows > 0;
}

void video_controller::update_recording_state()
{
    if (!m_main_record_button) return;
    
    gboolean recording_active = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_main_record_button));
    
    for (size_t i = 0; i < m_pipelines.size(); ++i) {
        if (i < m_checkboxes.size()) {
            gboolean selected = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_checkboxes[i]));
            m_pipelines[i]->set_recording(recording_active && selected);
        }
    }
}

void video_controller::on_window_destroy()
{
    if (m_quitting) return;
    m_open_windows--;
    if (m_open_windows <= 0) {
        m_quitting = true;
        m_pipelines.clear();
        gtk_main_quit();
    }
}

void video_controller::on_main_record_toggled()
{
    update_recording_state();
}

void video_controller::on_source_check_toggled()
{
    update_recording_state();
}

void video_controller::on_quit_clicked()
{
    m_quitting = true;
    m_pipelines.clear();
    gtk_main_quit();
}

void video_controller::on_dir_button_clicked()
{
    GtkWidget *dialog;
    GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER;
    gint res;

    dialog = gtk_file_chooser_dialog_new("Select Data Directory",
                                         GTK_WINDOW(m_control_window),
                                         action,
                                         "_Cancel",
                                         GTK_RESPONSE_CANCEL,
                                         "_Open",
                                         GTK_RESPONSE_ACCEPT,
                                         NULL);

    res = gtk_dialog_run(GTK_DIALOG(dialog));
    if (res == GTK_RESPONSE_ACCEPT)
    {
        char *filename;
        GtkFileChooser *chooser = GTK_FILE_CHOOSER(dialog);
        filename = gtk_file_chooser_get_filename(chooser);
        
        m_data_dir = filename;
        g_free(filename);
        
        std::cout << "Selected data directory: " << m_data_dir << std::endl;
        
        std::string label_text = "Data Directory: " + m_data_dir;
        gtk_label_set_text(GTK_LABEL(m_dir_label), label_text.c_str());

        // Check if currently recording
        gboolean was_recording = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_main_record_button));

        if (was_recording) {
            // Stop recording
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(m_main_record_button), FALSE);
            
            // Process pending events to ensure UI updates and stop commands are sent
            while (gtk_events_pending()) gtk_main_iteration();
        }

        // Update all pipelines with the new directory
        for (auto& pipe : m_pipelines) {
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
gboolean video_controller::restart_recording_cb(gpointer data)
{
    video_controller* self = static_cast<video_controller*>(data);
    if (self->m_main_record_button) {
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(self->m_main_record_button), TRUE);
    }
    return G_SOURCE_REMOVE;
}

void video_controller::on_window_destroy_cb(GtkWidget* widget, gpointer data)
{
    static_cast<video_controller*>(data)->on_window_destroy();
}

void video_controller::on_main_record_toggled_cb(GtkToggleButton* button, gpointer data)
{
    static_cast<video_controller*>(data)->on_main_record_toggled();
}

void video_controller::on_source_check_toggled_cb(GtkToggleButton* button, gpointer data)
{
    static_cast<video_controller*>(data)->on_source_check_toggled();
}

void video_controller::on_quit_clicked_cb(GtkButton* button, gpointer data)
{
    static_cast<video_controller*>(data)->on_quit_clicked();
}

void video_controller::on_dir_button_clicked_cb(GtkButton* button, gpointer data)
{
    static_cast<video_controller*>(data)->on_dir_button_clicked();
}
