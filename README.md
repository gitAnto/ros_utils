# ros_utils

ROS catkin package containing some simple but useful nodes.

Copyright (c) 2016

Antonio Coratelli,
Antonio Petitti


## License

BSD 3-Clause. See `LICENSE` file.



## Nodes

- [tf_republisher](#tf_republisher)
- [pcl_reframer](#pcl_reframer)
- [image_encoding_converter](#image_encoding_converter)
- [pose_noise_generator](#pose_noise_generator)
- [rgbd_throttle](#rgbd_throttle)
- [tf_link_map_to_world](#tf_link_map_to_world)
- [tf_eval](#tf_eval)

---

### tf_republisher
This node read messages from a `tf` topic and re-publishes particular transforms
from that topic to another one, eventually changing message `frame_id`s.

#### Subscribed Topics
- `tf_old` *tf::tfMessage*<br/>
  Input tf topic. Should usually be remapped to `/tf`.

#### Published Topics
- `tf_new` *tf::tfMessage*<br/>
  Output tf topic.

#### Params
- `original_frame_id` *string* (default "/world")
- `original_child_frame_id` *string* (default "/vicon/puma/puma")
- `new_frame_id` *string* (default "/world_new")
- `new_child_frame_id` *string* (default "/vicon/puma/puma_new")

---

### pcl_reframer
This node read `PointCloud2` messages from a topic and republishes it on anothre
topic changing the PointCloud `frame_id`.

#### Subscribed Topics
- `pcl_in` *sensor_msgs::PointCloud2*<br/>
  Input PointCloud topic.

#### Published Topics
- `pcl_out` *sensor_msgs::PointCloud2*<br/>
  Output PointCloud topic with `frame_id` changed.

#### Params
- `new_frame_id` *string* (default "/new_frame_id")

---

### image_encoding_converter
This node converts the encoding of an image topic to `BGR8`.

#### Subscribed Topics
- `image_in` *sensor_msgs::Image*<br/>
  Input image topic.

#### Published Topics
- `image_out` *sensor_msgs::Image*<br/>
  Output image topic.

---

### pose_noise_generator
This node adds noise to a topic.
Currently supports only *Gaussian* noise and the topics listed under `message_type` in *Param* section.

#### Subscribed Topics
- `in` *(programmable topic type, see Params)*<br/>
  Input topic without noise.

#### Published Topics
- `out` *(programmable topic type, see Params)*<br/>
  Output topic with noise.

#### Params
- `noise_type` *string* (default "Gaussian")<br/>
  - "Gaussian" noise Params:
    - `noise_mean` *double* (default "0.0")
    - `noise_stddev` *double* (default "1.0")
- `message_type` *string* (default "geometry_msgs/Pose")<br/>
  Can be:
  - `geometry_msgs/Pose`
  - `geometry_msgs/PoseStamped`
  - `geometry_msgs/Transform`
  - `geometry_msgs/TransformStamped`

---

### rgbd_throttle
This node limits the frame rate of an RGBD camera.

#### Subscribed Topics
- `rgb/info_in` *sensor_msgs::CameraInfo*<br/>
  RGB CameraInfo.
- `rgb/rect_in` *sensor_msgs::Image*<br/>
  RGB Image.
- `depth/info_in` *sensor_msgs::CameraInfo*<br/>
  Depth CameraInfo.
- `depth/rect_in` *sensor_msgs::Image*<br/>
  Depth Image.

#### Published Topics
- `rgb/info_out` *sensor_msgs::CameraInfo*<br/>
  Throttled RGB CameraInfo.
- `rgb/rect_out` *sensor_msgs::Image*<br/>
  Throttled RGB Image.
- `depth/info_out` *sensor_msgs::CameraInfo*<br/>
  Throttled Depth CameraInfo.
- `depth/rect_out` *sensor_msgs::Image*<br/>
  Throttled Depth Image.

#### Params
- `rate` *double* (default "5.0")<br/>
  Maximum framerate allowed.

---

### tf_link_map_to_world
This package links a disconnected sub-tree to the `/tf` root.
Suppose you have a `/tf` tree similar to this:
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
- `frame_world` *string* (default "/world")<br/>
  Frame attached to the absolute reference frame origin.
- `frame_groundtruth` *string* (default "/vicon/puma/puma")<br/>
  Frame attached to the ground truth pose of the robot.
- `frame_map` *string* (default "/map")<br/>
  Frame attached to the robot's relative reference frame origin.
- `rate` *double* (default "10.0")<br/>
  Frequency of the tf broadcaster.

---

### tf_eval
Evaluates the relative transform of between two frames and outputs it to a csv
file. Each line of the file contains (in order):
```
|| TIMESTAMP || ERROR       || GROUND TRUTH || ESTIMATE (Base Link) ||
||           || X Y Z R P Y || X Y Z R P Y  || X Y Z R P Y          ||
```

#### Params
- `frame_world` *string* (default "/world")<br/>
  Frame attached to the absolute reference frame origin.
- `frame_groundtruth` *string* (default "/vicon/puma/puma")<br/>
  Frame attached to the ground truth pose of the robot.
- `frame_baselink` *string* (default "/base_link")<br/>
  Frame attached to the robot, in its relative reference frame.
- `rate` *double* (default "10.0")<br/>
  Frequency of the tf listener.
- `csv_filename` *string* (default: "%Y_%m_%d_%H_%M_%S.csv")<br/>
  Output filename.
- `csv_delimiter` *string* (default: " ")<br/>
  Csv field delimiter.
