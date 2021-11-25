//
// Created by kehan on 2021/9/1.
//

#include "dv_ros/event_collectors.h"
#include "dv_ros/dynamic_configurator.h"

namespace dv_ros {

void Run(const std::string& config_file) {
  bool load_options_result = false;
  EventCollectorsOptions options;
  std::tie(load_options_result, options) =
      LoadOptions(config_file);
  if (!load_options_result) {
    ROS_ERROR("Load options failed");
    return;
  }
  EventCollectors event_collectors(options);
  DynamicConfigurator dynamic_configurator(event_collectors);
  ros::spin();
}

}  // namespace dv_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "dv_ros_node");
  dv_ros::Run(argv[1]);
  ros::shutdown();
}
