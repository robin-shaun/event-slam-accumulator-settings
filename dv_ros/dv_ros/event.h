//
// Created by kehan on 2021/9/1.
//

#ifndef DV_ROS_DV_ROS_EVENT_H_
#define DV_ROS_DV_ROS_EVENT_H_

#include <dvs_msgs/Event.h>
#include <celex5_msgs/EventVector.h>

#include "dv-sdk/data/event.hpp"

namespace dv_ros {

struct Event {
  uint32_t x = 0;
  uint32_t y = 0;
  ros::Time ts;
  bool polarity = false;
  uint16_t brightness = 0;      // only useful with celex event camera.
};

Event ToEvent(const dvs_msgs::Event& dvs_event);
Event ToEvent(const celex5_msgs::Event& celex_event);
dv::Event ToDVEvent(Event event);

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_EVENT_H_
