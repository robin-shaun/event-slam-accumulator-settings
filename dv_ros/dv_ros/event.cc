//
// Created by kehan on 2021/9/1.
//

#include "event.h"

namespace dv_ros {

Event ToEvent(const dvs_msgs::Event& dvs_event) {
  Event event;
  event.x = dvs_event.x;
  event.y = dvs_event.y;
  event.ts = dvs_event.ts;
  event.polarity = dvs_event.polarity;
  return event;
}

Event ToEvent(const celex5_msgs::Event& celex_event) {
  Event event;
  event.x = celex_event.y;
  event.y = celex_event.x;
  event.ts = ros::Time().fromNSec(celex_event.off_pixel_timestamp);
  event.polarity = celex_event.polarity;
  event.brightness = celex_event.brightness;
  return event;
}

dv::Event ToDVEvent(Event event) {
  return dv::Event(event.ts.toNSec(), event.x, event.y, event.polarity);
}

}  // namespace dv_ros
