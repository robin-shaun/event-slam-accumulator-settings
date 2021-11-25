//
// Created by kehan on 2021/9/1.
//

#ifndef DV_ROS_DV_ROS_EVENT_COLLECTOR_OPTIONS_H_
#define DV_ROS_DV_ROS_EVENT_COLLECTOR_OPTIONS_H_

#include <vector>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>

#include "dv_ros/accumulator/accumulator_options.h"

namespace dv_ros {

enum class EventDeviceType {
  DAVIS = 0,
  CELEX = 1
};

struct EventCollectorOptions {
  EventDeviceType device_type;
  std::string event_topic;
  AccumulatorOptions accumulator_options;
};

EventCollectorOptions CreateEventCollectorOptions(
    const cv::FileStorage& config_file_parser,
    size_t event_index);

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_EVENT_COLLECTOR_OPTIONS_H_
