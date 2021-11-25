//
// Created by kehan on 2021/9/1.
//

#include "dv_ros/accumulator/accumulator_options.h"

#include "dv_ros/utils/options_tools.h"

namespace dv_ros {

AccumulatorOptions CreateAccumulatorOptions(
    const cv::FileStorage& config_file_parser,
    size_t event_index) {
  AccumulatorOptions options{};
  options.frame_width =
      config_file_parser[eventI_frame_width(event_index)];
  options.frame_height =
      config_file_parser[eventI_frame_height(event_index)];
  options.accumulation_method =
      static_cast<AccumulationMethod>
      ((int) config_file_parser[eventI_accumulation_method(event_index)]);
  options.count_window_size =
      config_file_parser[eventI_count_window_size(event_index)];
  options.time_window_size =
      config_file_parser[eventI_time_window_size(event_index)];
  options.decay_function =
      static_cast<DecayFunction>
      ((int) config_file_parser[eventI_decay_function(event_index)]);
  options.decay_param =
      config_file_parser[eventI_decay_param(event_index)];
  options.min_potential =
      config_file_parser[eventI_min_potential(event_index)];
  options.max_potential =
      config_file_parser[eventI_max_potential(event_index)];
  options.neutral_potential =
      config_file_parser[eventI_neutral_potential(event_index)];
  options.event_contribution =
      config_file_parser[eventI_event_contribution(event_index)];
  options.rectify_polarity =
      config_file_parser[eventI_rectify_polarity(event_index)];
  options.synchronous_decay =
      config_file_parser[eventI_synchronous_decay(event_index)];
  config_file_parser[eventI_accumulated_frame_topic(event_index)]
      >> options.accumulated_frame_topic;
  options.no_motion_threshold =
      config_file_parser[eventI_no_motion_threshold(event_index)];
  options.use_knoise =
      config_file_parser[eventI_use_knoise(event_index)];
  if (options.use_knoise) {
    options.k_noise_options =
        CreateKNoiseOptions(config_file_parser, event_index);
  }
  return options;
}

}  // namespace dv_ros
