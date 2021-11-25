//
// Created by kehan on 2021/9/1.
//

#ifndef DV_ROS_DV_ROS_EVENT_COLLECTORS_H_
#define DV_ROS_DV_ROS_EVENT_COLLECTORS_H_

#include "dv_ros/event_collector.h"
#include "dv_ros/event_collectors_options.h"

namespace dv_ros {

class EventCollectors {
 public:
  explicit EventCollectors(EventCollectorsOptions options);
  const std::vector<std::shared_ptr<EventCollector>>& GetCollectors() const;
 private:
  EventCollectorsOptions options_;
  std::vector<std::shared_ptr<EventCollector>> collectors_;
};

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_EVENT_COLLECTORS_H_
