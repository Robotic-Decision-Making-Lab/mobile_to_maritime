# mobile_to_maritime

mobile_to_maritime is a collection of nodes and launch files for transforming
messages defined according the the coordinate frame conventions documented in
[REP-105](https://ros.org/reps/rep-0105.html) to the maritime conventions
documented in [REP-156](https://github.com/ros-infrastructure/rep/pull/398).

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
project, please refer to the demo launch file.

## License

mobile_to_maritime is released under the MIT license.
