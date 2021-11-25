//
// Created by kehan on 2021/9/1.
//

#include "dv_ros/event_collector.h"

#include <utility>
#include "dv_ros/event.h"
#include "utils/tic_toc.h"

namespace dv_ros {

EventCollector::EventCollector(EventCollectorOptions options)
    : options_(std::move(options)),
      nh_("~") {
  accumulator_ = std::make_shared<Accumulator>(options_.accumulator_options);
  if (options_.device_type == EventDeviceType::DAVIS) {
    subscriber_ =
        nh_.subscribe<dvs_msgs::EventArray>(
            options_.event_topic,
            2,
            &EventCollector::EventsCallback,
            this);
  } else if (options_.device_type == EventDeviceType::CELEX) {
    subscriber_ =
        nh_.subscribe<celex5_msgs::EventVector>(
            options_.event_topic,
            2,
            &EventCollector::EventsCallback,
            this);
  }
}

EventCollector::~EventCollector() = default;

void EventCollector::EventsCallback(
    const dvs_msgs::EventArrayConstPtr& events_msg) {
  TicToc tic_toc;
  static long num = 0;
  static double avg_time = 0;
  ProcessEvents(events_msg);
  auto current_time_cost = tic_toc.toc();
  avg_time += current_time_cost;
  num++;
  std::cout << "avg time cost: " << avg_time / num << " ms of " << num
            << " events packet "
            << "current time cost: " << current_time_cost << "ms "
            << "current events number: " << events_msg->events.size()
            << std::endl;
}

void EventCollector::EventsCallback(
    const celex5_msgs::EventVectorConstPtr& events_msg) {
  ProcessEvents(events_msg);
}

template <typename EventType>
void EventCollector::ProcessEvents(const EventType& events) {
  dv::EventStore dv_events_;
  if ((options_.accumulator_options.accumulation_method
      == AccumulationMethod::BY_COUNT ||
      options_.accumulator_options.accumulation_method
          == AccumulationMethod::BY_EVENTS_HZ_AND_COUNT) &&
      events->events.size()
          > options_.accumulator_options.count_window_size) {
    for (size_t i = events->events.size()
        - options_.accumulator_options.count_window_size;
         i < events->events.size(); ++i) {
      dv_events_.add(ToDVEvent(ToEvent(events->events.at(i))));
    }
  } else {
    for (auto event : events->events) {
      dv_events_.add(ToDVEvent(ToEvent(event)));
    }
  }
  accumulator_->AddNewEvents(dv_events_);
}

std::shared_ptr<Accumulator> EventCollector::GetMutableAccumulator() {
  return accumulator_;
}

}  // namespace dv_ros
