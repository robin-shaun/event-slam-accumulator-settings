//
// Created by kehan on 2021/9/1.
//

#ifndef DV_ROS_DV_ROS_ACCUMULATOR_ACCUMULATOR_H_
#define DV_ROS_DV_ROS_ACCUMULATOR_ACCUMULATOR_H_

#include <memory>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "dv-sdk/processing.hpp"

#include "dv_ros/accumulator/accumulator_options.h"
#include "dv_ros/knoise/k_noise.h"

namespace dv_ros {

class Accumulator {
 public:
  explicit Accumulator(const AccumulatorOptions& options);
  virtual ~Accumulator();
  void AddNewEvents(dv::EventStore& event_store);
  std::shared_ptr<AccumulatorOptions> GetMutableOptions();
  bool UpdateConfig();

 private:
  void DoPerFrameTime(const dv::EventStore& events);
  void DoPerEventNumber(const dv::EventStore& events);
  void DoPerAddEventData();
  void ElaborateFrame(const dv::EventStore& events);
  void PublishFrame();
  bool IsNoMotion(const dv::EventStore& events);

  std::shared_ptr<AccumulatorOptions> options_;
  cv::Mat corrected_frame_;
  dv::EventStreamSlicer slicer_;
  dv::Accumulator accumulator_;
  int64_t accumulation_time_ = -1;
  int64_t current_frame_time_ = -1;
  int slice_job_;
  dv::EventStore event_store_;

  ros::NodeHandle nh_;
  ros::Publisher accumulated_frame_pub_;

  std::shared_ptr<KNoise> k_noise_filter_;
};

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_ACCUMULATOR_ACCUMULATOR_H_
