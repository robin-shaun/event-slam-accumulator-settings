//
// Created by kehan on 2021/9/1.
//

#ifndef DV_ROS_DV_ROS_EVENT_COLLECTORS_OPTIONS_H_
#define DV_ROS_DV_ROS_EVENT_COLLECTORS_OPTIONS_H_

#include <vector>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#include "dv_ros/event_collector_options.h"

namespace dv_ros {

struct EventCollectorsOptions {
  int num_of_event_stream;
  std::vector<EventCollectorOptions> options;
};

EventCollectorsOptions CreateEventCollectorsOptions(
    const cv::FileStorage& config_file_parser);

std::tuple<bool, EventCollectorsOptions> LoadOptions(
    const boost::filesystem::path& config_file_path);

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_EVENT_COLLECTORS_OPTIONS_H_
