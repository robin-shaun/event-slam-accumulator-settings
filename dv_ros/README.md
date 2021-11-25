# dv_ros

> ROS package for iniVation Dynamic Vision System's dv-sdk.

Use [dv-sdk](https://inivation.gitlab.io/dv/dv-docs/docs/getting-started/) to process event streams in ROS.

## Feature

1. Use dv-sdk [`accumulator`](https://inivation.gitlab.io/dv/dv-docs/docs/accumulator-module/) functions in ROS.
   
2. Use `rqt_reconfigure` to modify the accumulating parameters online.

3. Support [`dvs_msgs/EventArray`](https://github.com/uzh-rpg/rpg_dvs_ros/blob/master/dvs_msgs/msg/EventArray.msg) and [`celex_msgs/EventVector`](https://github.com/kehanXue/CeleX5-ROS/blob/master/celex5_msgs/msg/EventVector.msg) event stream message.

4. Process multiple event streams in one ROS node, and config different parameters for each stream.

## Demo

<img src="assets/demo.gif" alt="demo" style="zoom:25%;" />

## Build 

1. Install the `iniVation dv-runtime` on your computer firstly. 
   
   - Binary installation steps can be found [here](https://inivation.gitlab.io/dv/dv-docs/docs/getting-started/#ubuntu-linux). 
     
   - For source code installation, you can follow the steps [here](https://gitlab.com/inivation/dv/dv-runtime). High version of fmt (>=7.0.3) library and Boost (>=1.73) library are required.

2. Clone the repository in your ROS Workspace, can then just build it.

## Usage

Use the prepared config file: davis240.yaml: 

```shell
roslaunch dv_ros davis240.launch
```

or

```shell
rosrun dv_ros dv_ros_node /path/to/the/davis240.yaml
```

## Parameters

You can process multiple event streams in one ros_node, 
the parameter `num_of_event_stream` specific the number of event streams you want to process.
The event stream can be capture from different device, such as DAVIS or CeleX.
Then you can config different parameters for each stream by using `eventN_` formulation.

The meaning of accumulate parameters can be found [here](https://inivation.gitlab.io/dv/dv-docs/docs/accumulator-module/).

Below is an example config file for stereo davis346 with different config:

```yaml
%YAML:1.0

num_of_event_stream: 2

event0_device_type: 0                   # 0: davis, 1: celex
event0_topic: "/dvs/left/events"
event0_frame_width: 346
event0_frame_height: 260
event0_accumulated_frame_topic: "/dvs/left/event_frame"  # accumulated frame topic's name, for publishing
event0_accumulation_method: 2     # 0: by time, 1: by count, 2: time window is sync with events data hz, by count
event0_count_window_size: 12000
event0_time_window_size: 33       # ms
event0_decay_function: 3          # 0: None, 1: Linear, 2: Exponential, 3: Step
event0_decay_param: 0.000001       # us
event0_min_potential: 0
event0_max_potential: 1
event0_neutral_potential: 0.5
event0_event_contribution: 0.15
event0_rectify_polarity: 0    # enable: 1, disable: 0
event0_synchronous_decay: 0   # enable: 1, disable: 0
event0_no_motion_threshold: 1000      # rate: events/s

event1_device_type: 0                   # 0: davis, 1: celex
event1_topic: "/dvs/right/events"
event1_frame_width: 346
event1_frame_height: 260
event1_accumulated_frame_topic: "/dvs/right/event_frame"  # accumulated frame topic's name, for publishing
event1_accumulation_method: 0     # 0: by time, 1: by count, 2: time window is sync with events data hz, by count
event1_count_window_size: 12000
event1_time_window_size: 33       # ms
event1_decay_function: 2          # 0: None, 1: Linear, 2: Exponential, 3: Step
event1_decay_param: 1000000       # us
event1_min_potential: 0
event1_max_potential: 0.3
event1_neutral_potential: 0
event1_event_contribution: 0.04
event1_rectify_polarity: 0    # enable: 1, disable: 0
event1_synchronous_decay: 0   # enable: 1, disable: 0
event1_no_motion_threshold: 1000          # rate: events/s
```
