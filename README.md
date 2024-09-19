# mobile_to_maritime

mobile_to_maritime is a convenience package used to transform messages defined
according the the coordinate frame conventions documented in
[REP-105](https://ros.org/reps/rep-0105.html) to the maritime conventions
documented in [REP-156](https://github.com/ros-infrastructure/rep/pull/398) and
back again.

## Supported message types

mobile_to_maritime can currently transform the following message types:

- `geometry_msgs/msg/Pose`
- `geometry_msgs/msg/PoseStamped`
- `geometry_msgs/msg/Twist`
- `geometry_msgs/msg/TwistStamped`
- `geometry_msgs/msg/Wrench`
- `geometry_msgs/msg/WrenchStamped`
- `nav_msgs/msg/Odometry`

If there is a standard message that you would like to be added, please consider
raising a new issue or submitting a pull request.

## Installation

To install mobile_to_maritime, first clone this project to the `src` directory
of your ROS workspace:

```bash
git clone git@github.com:Robotic-Decision-Making-Lab/mobile_to_maritime.git
```

After cloning the project, install the ROS dependencies using `rosdep`:

```bash
rosdep update && \
rosdep install -y --from-paths src --ignore-src
```

## Quick Start

To learn more about how to use the nodes and launch files provided in this
project, please refer to the [project demo](https://github.com/Robotic-Decision-Making-Lab/mobile_to_maritime/blob/main/mobile_to_maritime/launch/demo.launch.yaml).

## License

mobile_to_maritime is released under the MIT license.
