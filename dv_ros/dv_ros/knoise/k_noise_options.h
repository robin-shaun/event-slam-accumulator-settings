//
// Created by kehan on 2021/11/4.
//

#ifndef DV_ROS_DV_ROS_KNOISE_K_NOISE_OPTIONS_H_
#define DV_ROS_DV_ROS_KNOISE_K_NOISE_OPTIONS_H_

#include "dv-sdk/processing.hpp"

namespace dv_ros {

struct KNoiseOptions {
  int frame_width;
  int frame_height;
  int delta_t;
  size_t supporters;
};

KNoiseOptions CreateKNoiseOptions(const cv::FileStorage& config_file_parser,
                                  size_t event_index);

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_KNOISE_K_NOISE_OPTIONS_H_
