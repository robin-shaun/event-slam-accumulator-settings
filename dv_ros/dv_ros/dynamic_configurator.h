//
// Created by kehan on 2021/9/1.
//

#ifndef DV_ROS_DV_ROS_DYNAMIC_CONFIGURATOR_H_
#define DV_ROS_DV_ROS_DYNAMIC_CONFIGURATOR_H_

#include <vector>
#include <map>

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include "dv_ros/event_collectors.h"

namespace dv_ros {

class DynamicConfigurator {
 public:
  explicit DynamicConfigurator(const EventCollectors& collectors);
  virtual ~DynamicConfigurator();

 private:
  static void EventIAccumulationMethodCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      int new_value);
  static void EventICountWindowSizeCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      int new_value);
  static void EventITimeWindowSizeCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      int new_value);
  static void EventIDecayFunctionCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      int new_value);
  static void EventIDecayParamCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      std::string new_value);
  static void EventIMinPotentialCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      double new_value);
  static void EventIMaxPotentialCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      double new_value);
  static void EventINeutralPotentialCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      double new_value);
  static void EventIEventContributionCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      double new_value);
  static void EventIRectifyPolarityCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      bool new_value);
  static void EventISynchronousDecayCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      bool new_value);
  static void EventINoMotionCb(
      const std::shared_ptr<Accumulator>& accumulator,
      const std::shared_ptr<AccumulatorOptions>& options,
      int new_value);

  ros::NodeHandle nh_;
  ddynamic_reconfigure::DDynamicReconfigure ddr_;
};

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_DYNAMIC_CONFIGURATOR_H_
