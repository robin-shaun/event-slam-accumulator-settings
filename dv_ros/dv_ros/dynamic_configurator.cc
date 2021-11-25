//
// Created by kehan on 2021/9/1.
//

#include "dv_ros/dynamic_configurator.h"
#include "dv_ros/utils/options_tools.h"

namespace dv_ros {

DynamicConfigurator::DynamicConfigurator(const EventCollectors& collectors)
    : nh_("~"),
      ddr_(nh_) {
  std::map<std::string, int> accumulation_method_by_time_map = {
      {"BY_TIME", 0},
  };
  std::map<std::string, int> accumulation_method_by_count_map = {
      {"BY_COUNT", 1}
  };
  std::map<std::string, int> accumulation_method_by_hz_count_map = {
      {"BY_EVENTS_HZ_AND_COUNT", 2}
  };
  std::map<std::string, int> decay_function_map = {
      {"NONE", 0},
      {"LINEAR", 1},
      {"EXPONENTIAL", 2},
      {"STEP", 3}
  };
  size_t event_index = 0;
  for (auto& collector : collectors.GetCollectors()) {
    auto accumulator = collector->GetMutableAccumulator();
    auto option = accumulator->GetMutableOptions();
    // Register parameters to ROS parameters sever
    if (option->accumulation_method == AccumulationMethod::BY_TIME) {
      ddr_.registerEnumVariable<int>(
          eventI_accumulation_method(event_index),
          static_cast<int>(option->accumulation_method),
          boost::bind(&DynamicConfigurator::EventIAccumulationMethodCb,
                      accumulator,
                      option,
                      _1),
          "Method of accumulating events data",
          accumulation_method_by_time_map);
      ddr_.registerVariable<int>(
          eventI_time_window_size(event_index),
          option->time_window_size,
          boost::bind(&DynamicConfigurator::EventITimeWindowSizeCb,
                      accumulator,
                      option,
                      _1),
          "Time window for accumulating, unit. ms",
          1,
          100);
    } else if (option->accumulation_method == AccumulationMethod::BY_COUNT) {
      ddr_.registerEnumVariable<int>(
          eventI_accumulation_method(event_index),
          static_cast<int>(option->accumulation_method),
          boost::bind(&DynamicConfigurator::EventIAccumulationMethodCb,
                      accumulator,
                      option,
                      _1),
          "Method of accumulating events data",
          accumulation_method_by_count_map);
      ddr_.registerVariable<int>(
          eventI_count_window_size(event_index),
          option->count_window_size,
          boost::bind(&DynamicConfigurator::EventICountWindowSizeCb,
                      accumulator,
                      option,
                      _1),
          "Count window for accumulating, unit. num of event",
          1000,
          200000);
    } else if (option->accumulation_method
        == AccumulationMethod::BY_EVENTS_HZ_AND_COUNT) {
      ddr_.registerEnumVariable<int>(
          eventI_accumulation_method(event_index),
          static_cast<int>(option->accumulation_method),
          boost::bind(&DynamicConfigurator::EventIAccumulationMethodCb,
                      accumulator,
                      option,
                      _1),
          "Method of accumulating events data",
          accumulation_method_by_hz_count_map);
      ddr_.registerVariable<int>(
          eventI_count_window_size(event_index),
          option->count_window_size,
          boost::bind(&DynamicConfigurator::EventICountWindowSizeCb,
                      accumulator,
                      option,
                      _1),
          "Count window for accumulating, unit. num of event",
          1000,
          200000);
    }
    ddr_.registerEnumVariable<int>(
        eventI_decay_function(event_index),
        static_cast<int>(option->decay_function),
        boost::bind(&DynamicConfigurator::EventIDecayFunctionCb,
                    accumulator,
                    option,
                    _1),
        "Decay function for accumulating",
        decay_function_map);
    ddr_.registerVariable<std::string>(
        eventI_decay_param(event_index),
        std::to_string(option->decay_param),
        boost::bind(&DynamicConfigurator::EventIDecayParamCb,
                    accumulator,
                    option,
                    _1),
        "Decay function's parameter");
    ddr_.registerVariable<double>(
        eventI_min_potential(event_index),
        option->min_potential,
        boost::bind(&DynamicConfigurator::EventIMinPotentialCb,
                    accumulator,
                    option,
                    _1),
        "The resulting pixel value would be lower than Min potential, the value gets set to Min potential instead",
        0,
        1);
    ddr_.registerVariable<double>(
        eventI_max_potential(event_index),
        option->max_potential,
        boost::bind(&DynamicConfigurator::EventIMaxPotentialCb,
                    accumulator,
                    option,
                    _1),
        "The resulting pixel value would be higher than Max potential, the value gets set to Max potential instead",
        0,
        1);
    ddr_.registerVariable<double>(
        eventI_neutral_potential(event_index),
        option->neutral_potential,
        boost::bind(&DynamicConfigurator::EventINeutralPotentialCb,
                    accumulator,
                    option,
                    _1),
        "Neutral potential, just like the initial value of background",
        0,
        1);
    ddr_.registerVariable<double>(
        eventI_event_contribution(event_index),
        option->event_contribution,
        boost::bind(&DynamicConfigurator::EventIEventContributionCb,
                    accumulator,
                    option,
                    _1),
        "The contribution an event has onto the image. "
        "If an event arrives at a position x, y, the pixel value in the frame "
        "at x, y gets increased / decreased by the value of Event contribution,"
        " based on the events polarity.",
        0,
        1);
    ddr_.registerVariable<bool>(
        eventI_rectify_polarity(event_index),
        option->rectify_polarity,
        boost::bind(&DynamicConfigurator::EventIRectifyPolarityCb,
                    accumulator,
                    option,
                    _1),
        "The event polarity is negative, and Rectify polarity is enabled, "
        "then the event is counted positively");
    ddr_.registerVariable<bool>(
        eventI_synchronous_decay(event_index),
        option->synchronous_decay,
        boost::bind(&DynamicConfigurator::EventISynchronousDecayCb,
                    accumulator,
                    option,
                    _1),
        "If this value is set, decay happens in continous time for all pixels.");
    ddr_.registerVariable<int>(
        eventI_no_motion_threshold(event_index),
        option->no_motion_threshold,
        boost::bind(&DynamicConfigurator::EventINoMotionCb,
                    accumulator,
                    option,
                    _1),
        "If the events\' speed is lower than this value, "
        "do not add new events in the accumulator. units events/s",
        0,
        30000);
    ++event_index;
  }
  ddr_.publishServicesTopics();
}

DynamicConfigurator::~DynamicConfigurator() = default;

void DynamicConfigurator::EventIAccumulationMethodCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    int new_value) {
  options->accumulation_method = static_cast<AccumulationMethod>(new_value);
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventICountWindowSizeCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    int new_value) {
  options->count_window_size = new_value;
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventITimeWindowSizeCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    int new_value) {
  options->time_window_size = new_value;
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventIDecayFunctionCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    int new_value) {
  options->decay_function = static_cast<DecayFunction>(new_value);
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventIDecayParamCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    std::string new_value) {
  char* tmp;
  options->decay_param = strtod(new_value.c_str(), &tmp);
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventIMinPotentialCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    double new_value) {
  options->min_potential = new_value;
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventIMaxPotentialCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    double new_value) {
  options->max_potential = new_value;
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventINeutralPotentialCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    double new_value) {
  options->neutral_potential = new_value;
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventIEventContributionCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    double new_value) {
  options->event_contribution = new_value;
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventIRectifyPolarityCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    bool new_value) {
  options->rectify_polarity = new_value;
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventISynchronousDecayCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<AccumulatorOptions>& options,
    bool new_value) {
  options->synchronous_decay = new_value;
  accumulator->UpdateConfig();
}

void DynamicConfigurator::EventINoMotionCb(
    const std::shared_ptr<Accumulator>& accumulator,
    const std::shared_ptr<
        AccumulatorOptions>& options,
    int new_value) {
  options->no_motion_threshold = new_value;
}

}  // namespace dv_ros
