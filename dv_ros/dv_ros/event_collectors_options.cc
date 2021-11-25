//
// Created by kehan on 2021/9/1.
//

#include "dv_ros/event_collectors_options.h"

#include <ros/ros.h>

namespace dv_ros {

EventCollectorsOptions CreateEventCollectorsOptions(
    const cv::FileStorage& config_file_parser) {
  EventCollectorsOptions options;
  options.num_of_event_stream =
      config_file_parser["num_of_event_stream"];
  for (size_t i = 0; i < options.num_of_event_stream; ++i) {
    options.options.emplace_back(
        CreateEventCollectorOptions(config_file_parser, i)
    );
  }
  return options;
}

std::tuple<bool, EventCollectorsOptions> LoadOptions(
    const boost::filesystem::path& config_file_path) {
  ROS_INFO("Load config file: %s",
           config_file_path.string().c_str());
  if (!boost::filesystem::is_regular_file(config_file_path)) {
    ROS_ERROR("Selected config file is not a regular file!");
    return std::make_tuple(false, EventCollectorsOptions());
  }
  cv::FileStorage
      config_parser(config_file_path.string(), cv::FileStorage::READ);
  if (!config_parser.isOpened()) {
    ROS_ERROR("Parser open config file error!");
    return std::make_tuple(false, EventCollectorsOptions());
  }
  try {
    auto options = CreateEventCollectorsOptions(config_parser);
    return std::make_tuple(true, options);
  } catch (std::exception& e) {
    ROS_ERROR("Parse config file error! Please check your "
              "config file carefully.");
    return std::make_tuple(false, EventCollectorsOptions());
  }
}

}  // namespace dv_ros
