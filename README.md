# CNS Flight Stack: Nodelet Rosbag ROS1 Package (nodelet_rosbag)

[![License](https://img.shields.io/badge/License-APACHE--2.0-green.svg)](./LICENSE) [![Paper](https://img.shields.io/badge/IEEEXplore-10.1109/LRA.2022.3196117-00629B.svg?logo=ieee)](https://doi.org/10.1109/LRA.2022.3196117) [![Release](https://img.shields.io/github/v/release/aau-cns/nodelet_rosbag?include_prereleases&logo=github)](https://github.com/aau-cns/nodelet_rosbag/releases)

Maintainer: [Martin Scheiber](mailto:martin.scheiber@aau.at)

This is a modified fork of the [osrf/nodelet_rosbag](https://github.com/osrf/nodelet_rosbag) as used within the [CNS Flight Stack] for recording data in rosbags using nodelets (direct transfer via memeory rather than TCP/IP) wherever possible (depending on the "sender").

## Credit
This code was written by the [Control of Networked System (CNS)](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/), University of Klagenfurt, Klagenfurt, Austria.

## License
This software is made available to the public to use, licensed under the terms of the APACHE-2.0-License, the full terms of which are made available in the `LICENSE` file.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding [academic paper] and consult the `LICENSE` file for a detailed explanation.

```latex
@article{cns_flightstack22,
    title        = {Flight Stack for Reproducible and Customizable Autonomy Applications in Research and Industry},
    author       = {Scheiber, Martin and Fornasier, Alessandro and Jung, Roland and Böhm, Christoph and Dhakate, Rohit and Stewart, Christian and Steinbrener, Jan and Weiss, Stephan and Brommer, Christian},
    journal      = {IEEE Robotics and Automation Letters},
    volume       = {7},
    number       = {4},
    year         = {2022},
    doi          = {10.1109/LRA.2022.3196117},
    url          = {https://ieeexplore.ieee.org/document/9849131},
    pages        = {11283--11290}
}
```

## Building the package
```bash
catkin build nodelet_rosbag
```

## Run the nodelet_rosbag

### Using the launch file

```bash
roslaunch nodelet_rosbag nodelet_rosbag.launch start_manager:=True # if you want to start the manager
roslaunch nodelet_rosbag nodelet_rosbag.launch nodelet_manager_name:=<manager_name> # if another manager already exists
```

#### Default Launchfile Parameters

The default parameters are setup for camera recordings, as nodelets have the most performance impact on images.

| ROS parameter | description | default value |
|:-------------:|:-----------:|:-------------:|
| `rosbag_path` | path to record the bagfile(s) to | `${HOME}/recordings` |
| `rosbag_prefix` | prefix of the recorded file | `cam` |
| `rosbag_topics` | array of topics to record | `[/mission_cam/image_raw]` |
| `start_manager` | starts the nodelet manager | `false` |
| `nodelet_manager_name` | name of the ROS nodelet manager | `nodelet_manager` |
| `respawn` | autorespawn the manager and nodelet if they die or are killed | `false` |


### Using individual commands
Read the [official document on running a nodelet](http://wiki.ros.org/nodelet/Tutorials/Running%20a%20nodelet).

#### TAB1:
```bash
roscore
```

#### TAB2:
```bash
source devel/setup.bash
rosrun nodelet nodelet manager __name:=nodelet_manager
```

#### TAB3:

Start the `nodelet_rosbag` by specifying the `rosbag_path` and a list of topics to record `rosbag_topics`.

```bash
source devel/setup.bash
roslaunch nodelet_rosbag nodelet_rosbag.launch rosbag_path:=/tmp rosbag_topics:="[/mission_cam/image_raw,/mission_cam/image_rect,/mission_cam/rgb]"
```

---

Copyright (C) on changes 2021-2023 Christian Brommer, Roland Jung, Alessandro Fornasier, and Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
You can contact the authors at [christian.brommer@aau.at](mailto:christian.brommer@aau.at?subject=[CNS%20Flight%20Stack]%20nodelet_rosbag%20package), [roland.jung@aau.at](mailto:roland.jung@aau.at?subject=[CNS%20Flight%20Stack]%20nodelet_rosbag%20package), [alessandro.fornasier@aau.at](mailto:alessandro.fornasier@aau.at?subject=[CNS%20Flight%20Stack]%20nodelet_rosbag%20package), [martin.scheiber@aau.at](mailto:martin.scheiber@aau.at?subject=[CNS%20Flight%20Stack]%20nodelet_rosbag%20package).

[CNS Flight Stack]: https://github.com/aau-cns/flight_stack
[academic paper]: https://ieeexplore.ieee.org/document/9849131
