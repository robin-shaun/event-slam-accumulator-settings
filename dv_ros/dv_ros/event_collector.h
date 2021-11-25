//
// Created by kehan on 2021/9/1.
//

#ifndef DV_ROS_DV_ROS_EVENT_COLLECTOR_H_
#define DV_ROS_DV_ROS_EVENT_COLLECTOR_H_

#include <memory>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <celex5_msgs/EventVector.h>

#include "dv-sdk/data/event.hpp"

#include "dv_ros/event_collector_options.h"
#include "dv_ros/accumulator/accumulator.h"

namespace dv_ros {

class EventCollector {
 public:
  explicit EventCollector(EventCollectorOptions options);
  virtual ~EventCollector();
  std::shared_ptr<Accumulator> GetMutableAccumulator();

 private:
  void EventsCallback(const dvs_msgs::EventArrayConstPtr& events_msg);
  void EventsCallback(const celex5_msgs::EventVectorConstPtr& events_msg);

  template <typename EventType>
  void ProcessEvents(const EventType& events);

  EventCollectorOptions options_;
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::shared_ptr<Accumulator> accumulator_;
};

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_EVENT_COLLECTOR_H_
