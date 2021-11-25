//
// Created by kehan on 2021/9/1.
//

#include "dv_ros/event_collector_options.h"

#include "dv_ros/utils/options_tools.h"

namespace dv_ros {

EventCollectorOptions CreateEventCollectorOptions(
    const cv::FileStorage& config_file_parser,
    size_t event_index) {
  EventCollectorOptions options;
  options.device_type =
      static_cast<EventDeviceType>(
          static_cast<int>(config_file_parser[eventI_device_type(event_index)]));
  config_file_parser[eventI_topic(event_index)] >> options.event_topic;
  options.accumulator_options =
      CreateAccumulatorOptions(config_file_parser, event_index);
  return options;
}

}  // namespace dv_ros
