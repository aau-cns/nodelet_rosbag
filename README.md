# CNS Flight Stack: Nodelet Rosbag ROS1 Package (nodelet_rosbag)

[![License](https://img.shields.io/badge/License-APACHE--2.0-green.svg)](./LICENSE)

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

## Run the nodelet_rosbag

Read the [official document on running a nodelet](http://wiki.ros.org/nodelet/Tutorials/Running%20a%20nodelet).

### TAB1:
```
$ roscore
```

### TAB2:
```
$ source devel/setup.bash
$ rosrun nodelet nodelet manager __name:=nodelet_manager
```

### TAB3:

Start the `nodelet_rosbag` by specifying the `rosbag_path` and a list of topics to record `rosbag_topics`.
```
$ source devel/setup.bash
$ roslaunch nodelet_rosbag nodelet_rosbag.launch rosbag_path:=/tmp rosbag_topics:="[/mission_cam/image_raw,/mission_cam/image_rect,/mission_cam/rgb]"
```

[CNS Flight Stack]: https://github.com/aau-cns/flight_stack
[academic paper]: https://ieeexplore.ieee.org/document/9849131
