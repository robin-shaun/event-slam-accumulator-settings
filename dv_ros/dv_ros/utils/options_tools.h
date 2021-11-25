//
// Created by kehan on 2021/9/1.
//

#ifndef DV_ROS_DV_ROS_UTILS_OPTIONS_TOOLS_H_
#define DV_ROS_DV_ROS_UTILS_OPTIONS_TOOLS_H_

#include <string>

inline std::string eventI_device_type(size_t event_index) {
  return "event" + std::to_string(event_index) + "_device_type";
}

inline std::string eventI_topic(size_t event_index) {
  return "event" + std::to_string(event_index) + "_topic";
}

inline std::string eventI_accumulated_frame_topic(size_t event_index) {
  return "event" + std::to_string(event_index) + "_accumulated_frame_topic";
}

inline std::string eventI_no_motion_threshold(size_t event_index) {
  return "event" + std::to_string(event_index) + "_no_motion_threshold";
}

inline std::string eventI_frame_width(size_t event_index) {
  return "event" + std::to_string(event_index) + "_frame_width";
}

inline std::string eventI_frame_height(size_t event_index) {
  return "event" + std::to_string(event_index) + "_frame_height";
}

inline std::string eventI_count_window_size(size_t event_index) {
  return "event" + std::to_string(event_index) + "_count_window_size";
}

inline std::string eventI_accumulation_method(size_t event_index) {
  return "event" + std::to_string(event_index) + "_accumulation_method";
}

inline std::string eventI_time_window_size(size_t event_index) {
  return "event" + std::to_string(event_index) + "_time_window_size";
}

inline std::string eventI_decay_function(size_t event_index) {
  return "event" + std::to_string(event_index) + "_decay_function";
}

inline std::string eventI_decay_param(size_t event_index) {
  return "event" + std::to_string(event_index) + "_decay_param";
}

inline std::string eventI_min_potential(size_t event_index) {
  return "event" + std::to_string(event_index) + "_min_potential";
}

inline std::string eventI_max_potential(size_t event_index) {
  return "event" + std::to_string(event_index) + "_max_potential";
}

inline std::string eventI_neutral_potential(size_t event_index) {
  return "event" + std::to_string(event_index) + "_neutral_potential";
}

inline std::string eventI_event_contribution(size_t event_index) {
  return "event" + std::to_string(event_index) + "_event_contribution";
}

inline std::string eventI_rectify_polarity(size_t event_index) {
  return "event" + std::to_string(event_index) + "_rectify_polarity";
}

inline std::string eventI_synchronous_decay(size_t event_index) {
  return "event" + std::to_string(event_index) + "_synchronous_decay";
}

inline std::string eventI_knoise_delta_t(size_t event_index) {
  return "event" + std::to_string(event_index) + "_knoise_delta_t";
}

inline std::string eventI_knoise_supporters(size_t event_index) {
  return "event" + std::to_string(event_index) + "_knoise_supporters";
}

inline std::string eventI_use_knoise(size_t event_index) {
  return "event" + std::to_string(event_index) + "_use_knoise";
}

#endif //DV_ROS_DV_ROS_UTILS_OPTIONS_TOOLS_H_
