# ros_utils

ROS catkin package containing some simple but useful nodes.



## License

BSD 3-Clause. See `LICENSE` file.



## Nodes

- [rgbd_throttle](#rgbd_throttle)
- [tf_link_map_to_world](#tf_link_map_to_world)
- [tf_eval](#tf_eval)


### rgbd_throttle
This node limits the frame rate of an RGBD camera.

#### Subscribed Topics
- `rgb/info_in` *sensor_msgs::CameraInfo*<br/>
RGB CameraInfo.
- `rgb/rect_in` *sensor_msgs::Image*
RGB Image.
- `depth/info_in` *sensor_msgs::CameraInfo*
Depth CameraInfo.
- `depth/rect_in` *sensor_msgs::Image*
Depth Image.

#### Published Topics
- `rgb/info_out` *sensor_msgs::CameraInfo*
Throttled RGB CameraInfo.
- `rgb/rect_out` *sensor_msgs::Image*
Throttled RGB Image.
- `depth/info_out` *sensor_msgs::CameraInfo*
Throttled Depth CameraInfo.
- `depth/rect_out` *sensor_msgs::Image*
Throttled Depth Image.

#### Params
- `rate` *double* (default `5.0`)
Maximum framerate allowed.


### tf_link_map_to_world
This package links a disconnected sub-tree to the `/tf` root.
Suppose you have a /tf tree similar to this:
```
/world +
       |--- /vicon/robot_name
       |--- /vicon/some_other_object
       |--- ...
/map +
     |--- /odom +
                |--- /base_link
```
If you want to link the `/map` frame to the `/world` frame, and the transform
between `/world` and `/map` is given by the first transform from
`/world` to `/vicon/robot_name`, you can use this node.

#### Params
- `frame_world` *string* (default `/world`)
Frame attached to the absolute reference frame origin.
- `frame_groundtruth` *string* (default `/vicon/puma/puma`)
Frame attached to the ground truth pose of the robot.
- `frame_map` *string* (default `/map`)
Frame attached to the robot's relative reference frame origin.
- `rate` *double* (default `10.0`)
Frequency of the tf broadcaster.


### tf_eval
Evaluates the relative transform of between two frames and outputs it to a csv
file. Each line of the file contains (in order):
```
|| TIMESTAMP || ERROR       || GROUND TRUTH || ESTIMATE (Base Link) ||
||           || X Y Z R P Y || X Y Z R P Y  || X Y Z R P Y          ||
```

#### Params
- `frame_world` *string* (default `/world`)
Frame attached to the absolute reference frame origin.
- `frame_groundtruth` *string* (default `/vicon/puma/puma`)
Frame attached to the ground truth pose of the robot.
- `frame_baselink` *string* (default `/base_link`)
Frame attached to the robot, in its relative reference frame.
- `rate` *double* (default `10.0`)
Frequency of the tf listener.
- `csv_filename` *string* (default: "%Y_%m_%d_%H_%M_%S.csv")
Output filename.
- `csv_delimiter` *string* (default: " ")
Csv field delimiter.

