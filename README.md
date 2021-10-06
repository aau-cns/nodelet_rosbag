# nodelet_rosbag


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

