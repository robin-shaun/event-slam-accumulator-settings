//
// Created by kehan on 2021/11/4.
//

#include "dv_ros/knoise/k_noise_options.h"
#include "dv_ros/utils/options_tools.h"

namespace dv_ros {

KNoiseOptions CreateKNoiseOptions(const cv::FileStorage& config_file_parser,
                                  size_t event_index) {
  KNoiseOptions options{};
  options.frame_width =
      config_file_parser[eventI_frame_width(event_index)];
  options.frame_height =
      config_file_parser[eventI_frame_height(event_index)];
  options.delta_t =
      config_file_parser[eventI_knoise_delta_t(event_index)];
  options.supporters =
      int(config_file_parser[eventI_knoise_supporters(event_index)]);
  return options;
}

}  // namespace dv_ros