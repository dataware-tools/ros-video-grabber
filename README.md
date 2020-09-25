# ros-video-grabber

This is a package of ROS nodes for grabbing videos.

## Requirements
- A video file
- A csv file containing list of timestamps of the video frames

Note that the number of frames in the video and rows in the csv file must be the same.

## Usage
### 1. Make
```bash
$ cd src
$ catkin_init_workspace
$ cd ..
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make

```

### 2. Source setup.bash
```bash
$ source devel/setup.bash

```

### 3. Execute video-grabber with either rosrun or roslaunch

#### 3.1 Execute video-grabber with rosrun
```bash
$ rosrun video_grabber video_grabber_node \
    _path_to_video:=<path to video> \
    _path_to_timestamp:=<path to timestamp csv> \
    _frame_id:=<frame id to publish (default: camera)> \
    _seek_frequency:=<refresh rate (should be higher than video FPS. default: 60)>

```

#### 3.1 Execute video-grabber with rosrun
```bash
$ roslaunch video_grabber sample.launch \
    path_to_video:=<path to video> \
    path_to_timestamp:=<path to timestamp csv> \
    frame_id:=<frame id to publish (default: camera)> \
    seek_frequency:=<refresh rate (should be higher than video FPS. default: 60)>

```

Video frames will be published with topic name `/video_grabber` .
