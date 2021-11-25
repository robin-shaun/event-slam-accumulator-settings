//
// Created by kehan on 2021/9/1.
//

#include "dv_ros/accumulator/accumulator.h"
#include "dv_ros/utils/tic_toc.h"

namespace dv_ros {

Accumulator::Accumulator(const AccumulatorOptions& options)
    : nh_("~") {
  options_ = std::make_shared<AccumulatorOptions>(options);
  if (options_->use_knoise) {
    k_noise_filter_ = std::make_shared<KNoise>(options_->k_noise_options);
  }
  accumulated_frame_pub_ =
      nh_.advertise<sensor_msgs::Image>(options_->accumulated_frame_topic, 2);
  accumulator_ =
      dv::Accumulator(cv::Size(options_->frame_width, options_->frame_height),
                      static_cast<dv::Accumulator::Decay>(options_->decay_function),
                      options_->decay_param,
                      options_->synchronous_decay,
                      options_->event_contribution,
                      options_->max_potential,
                      options_->neutral_potential,
                      options_->min_potential,
                      options_->rectify_polarity);
  if (options_->accumulation_method == AccumulationMethod::BY_TIME) {
    accumulation_time_ = options_->time_window_size * 1e6;
    slice_job_ =
        slicer_.doEveryTimeInterval(
            accumulation_time_,
            std::function<void(const dv::EventStore&)>(
                std::bind(&Accumulator::DoPerFrameTime,
                          this,
                          std::placeholders::_1)));
  } else if (options_->accumulation_method == AccumulationMethod::BY_COUNT) {
    slice_job_ =
        slicer_.doEveryNumberOfEvents(
            options_->count_window_size,
            std::function<void(const dv::EventStore&)>(
                std::bind(&Accumulator::DoPerEventNumber,
                          this,
                          std::placeholders::_1)));
  }
}

Accumulator::~Accumulator() = default;

void Accumulator::AddNewEvents(dv::EventStore& event_store) {
  if (IsNoMotion(event_store)) {
    return;
  }
  if (options_->accumulation_method == AccumulationMethod::BY_TIME ||
      options_->accumulation_method == AccumulationMethod::BY_COUNT) {
    slicer_.accept(event_store);
  } else if (options_->accumulation_method
      == AccumulationMethod::BY_EVENTS_HZ_AND_COUNT) {
    event_store_.add(event_store);
    if (options_->use_knoise) {
      k_noise_filter_->ProcessEvents(event_store_);
    }
    DoPerAddEventData();
  } else {
    ROS_ERROR("Unrecognized accumulation method selected");
  }
}

void Accumulator::DoPerFrameTime(const dv::EventStore& events) {
  if (current_frame_time_ < 0) {
    current_frame_time_ = events.getLowestTime();
  }
  current_frame_time_ += accumulation_time_;
  ElaborateFrame(events);
}

void Accumulator::DoPerEventNumber(const dv::EventStore& events) {
  current_frame_time_ = events.getHighestTime();
  ElaborateFrame(events);
}

void Accumulator::DoPerAddEventData() {
  if (event_store_.getTotalLength() <= options_->count_window_size) {
    current_frame_time_ = event_store_.getHighestTime();
    ElaborateFrame(event_store_);
  } else {
    event_store_ = event_store_.slice(
        event_store_.getTotalLength() - options_->count_window_size,
        options_->count_window_size);
    current_frame_time_ = event_store_.getHighestTime();
    ElaborateFrame(event_store_);
  }
}

void Accumulator::ElaborateFrame(const dv::EventStore& events) {
  // if (options_->use_knoise) {
  //   dv::EventStore event_store = events;
  //   k_noise_filter_->ProcessEvents(event_store);
  //   accumulator_.accumulate(event_store);
  // } else {
  // }
  accumulator_.accumulate(events);
  // generate frame
  auto frame = accumulator_.generateFrame();
  // make sure frame is in correct exposure and data type
  double scale_factor
      = 255.0 / static_cast<double>(accumulator_.getMaxPotential()
          - accumulator_.getMinPotential());
  double shift_factor =
      -static_cast<double>(accumulator_.getMinPotential()) * scale_factor;
  frame.convertTo(corrected_frame_, CV_8U, scale_factor, shift_factor);

  PublishFrame();
}

void Accumulator::PublishFrame() {
  auto frame = cv_bridge::CvImage(std_msgs::Header(),
                                  "mono8",
                                  corrected_frame_).toImageMsg();
  frame->header.stamp = ros::Time().fromNSec(current_frame_time_);
  accumulated_frame_pub_.publish(frame);
}

bool Accumulator::IsNoMotion(const dv::EventStore& events) {
  if (events.isEmpty()) {
    return true;
  }
  auto event_rate = 1.0 * events.getTotalLength() /
      ((events.back().timestamp() - events.front().timestamp()) / 1e9);
  if (event_rate < options_->no_motion_threshold) {
    ROS_INFO("The event camera doesn't move this time...");
    return true;
  }
  return false;
}

std::shared_ptr<AccumulatorOptions> Accumulator::GetMutableOptions() {
  return options_;
}

bool Accumulator::UpdateConfig() {
  if (options_->accumulation_method == AccumulationMethod::BY_TIME) {
    slicer_.modifyTimeInterval(slice_job_,
                               options_->time_window_size * 1e6);
  } else if (options_->accumulation_method == AccumulationMethod::BY_COUNT) {
    slicer_.modifyNumberInterval(slice_job_,
                                 options_->count_window_size);
  }
  accumulator_.setDecayFunction(
      static_cast<dv::Accumulator::Decay>(options_->decay_function));
  accumulator_.setDecayParam(options_->decay_param);
  accumulator_.setMinPotential(options_->min_potential);
  accumulator_.setMaxPotential(options_->max_potential);
  accumulator_.setNeutralPotential(options_->neutral_potential);
  accumulator_.setEventContribution(options_->event_contribution);
  accumulator_.setRectifyPolarity(options_->rectify_polarity);
  accumulator_.setSynchronousDecay(options_->synchronous_decay);
  return true;
}

}  // namespace dv_ros
