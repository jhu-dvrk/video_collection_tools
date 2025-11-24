// Author(s): Anton Deguet
// Copyright 2025 Johns Hopkins University

#include "video_controller.h"
#include <fstream>
#include <gst/gst.h>
#include <gtk/gtk.h>
#include <iostream>
#include <json/json.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>

class VideoRecorderNode : public rclcpp::Node {
public:
  VideoRecorderNode(video_controller *controller)
      : Node("video_recorder_node"), controller_(controller) {
    set_record_sub_ = this->create_subscription<std_msgs::msg::String>(
        "set_record", 10,
        std::bind(&VideoRecorderNode::set_record_callback, this,
                  std::placeholders::_1));
    unset_record_sub_ = this->create_subscription<std_msgs::msg::String>(
        "unset_record", 10,
        std::bind(&VideoRecorderNode::unset_record_callback, this,
                  std::placeholders::_1));

    start_record_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "start_recording", 10,
        std::bind(&VideoRecorderNode::start_record_callback, this,
                  std::placeholders::_1));
    stop_record_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "stop_recording", 10,
        std::bind(&VideoRecorderNode::stop_record_callback, this,
                  std::placeholders::_1));
    set_data_dir_sub_ = this->create_subscription<std_msgs::msg::String>(
        "set_data_directory", 10,
        std::bind(&VideoRecorderNode::set_data_dir_callback, this,
                  std::placeholders::_1));
  }

private:
  void set_record_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (controller_) {
      controller_->set_recording_by_name(msg->data, true);
    }
  }
  void unset_record_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (controller_) {
      controller_->set_recording_by_name(msg->data, false);
    }
  }
  void start_record_callback(const std_msgs::msg::Empty::SharedPtr) {
    // Start recording for all pipelines
    if (controller_) {
      controller_->set_recording_by_name("", true); // Empty name means all
    }
  }
  void stop_record_callback(const std_msgs::msg::Empty::SharedPtr) {
    // Stop recording for all pipelines
    if (controller_) {
      controller_->set_recording_by_name("", false); // Empty name means all
    }
  }
  void set_data_dir_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (controller_) {
      // Set data directory for all pipelines
      controller_->set_data_directory(msg->data);
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_record_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr unset_record_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_record_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_record_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_data_dir_sub_;
  video_controller *controller_;
};

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

  // ROS2 node setup
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<VideoRecorderNode>(&controller);
  std::thread ros_spin_thread([ros_node]() { rclcpp::spin(ros_node); });

  if (!controller.has_open_windows()) {
    std::cerr << "No windows created. Exiting." << std::endl;
    rclcpp::shutdown();
    ros_spin_thread.join();
    return 0;
  }

  // Run GTK main loop
  gtk_main();

  rclcpp::shutdown();
  ros_spin_thread.join();

  return 0;
}
