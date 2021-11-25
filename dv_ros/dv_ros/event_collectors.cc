//
// Created by kehan on 2021/9/1.
//

#include "dv_ros/event_collectors.h"

#include <utility>

namespace dv_ros {

EventCollectors::EventCollectors(EventCollectorsOptions options)
    : options_(std::move(options)) {
  for (int i = 0; i < options_.num_of_event_stream; ++i) {
    auto collector =
        std::make_shared<EventCollector>(options_.options.at(i));
    collectors_.emplace_back(collector);
  }
}

const std::vector<std::shared_ptr<EventCollector>>&
EventCollectors::GetCollectors() const {
  return collectors_;
}

}  // namespace dv_ros
