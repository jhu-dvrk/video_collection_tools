// Author(s): Anton Deguet
// Copyright 2025 Johns Hopkins University

#include "video_controller.h"
#include <fstream>
#include <gst/gst.h>
#include <gtk/gtk.h>
#include <iostream>
#include <json/json.h>
#include <memory>
#include <stdexcept>

int main(int argc, char *argv[]) {
  // Initialize GTK
  gtk_init(&argc, &argv);

  // Initialize GStreamer
  gst_init(&argc, &argv);

  // Parse command line arguments
  std::string config_file = "config.json"; // Default config file
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "-c" && i + 1 < argc) {
      config_file = argv[i + 1];
      i++;
    }
  }

  Json::Value root;
  std::ifstream file(config_file);
  if (file.is_open()) {
    Json::Reader reader;
    if (reader.parse(file, root)) {
      std::cout << "Loaded configuration from " << config_file << std::endl;
    } else {
      std::cerr << "Failed to parse configuration file: "
                << reader.getFormattedErrorMessages() << std::endl;
      return 1;
    }
  } else {
    std::cerr << "Failed to open configuration file: " << config_file
              << std::endl;
    return 1;
  }

  video_controller controller(root);
  controller.create_ui();

  if (!controller.has_open_windows()) {
    std::cerr << "No windows created. Exiting." << std::endl;
    return 0;
  }

  // Run GTK main loop
  gtk_main();
  return 0;
}
