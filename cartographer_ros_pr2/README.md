## About this package
Test of SLAM based on [Cartographer](https://github.com/googlecartographer) for PR2

## Install cartgrapher
Obtain the latest cartographer from [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html).


### version requirement
- cartographer: `master` (**note**: please checkout to `master` branch (not `v1.0.0`), and follow [this instruction](https://gist.github.com/tongtybj/c5e0cec0160c2194298ee4b5895c3753))
- cartographer_ros: [`tongtybj:set_initpose_from_rviz`](https://github.com/tongtybj/cartographer_ros/tree/set_initpose_from_rviz)  (opening PR [#1284](https://github.com/googlecartographer/cartographer_ros/pull/1284))
- cartographer_ :`master` in [here](https://github.com/googlecartographer/cartographer_turtlebot) 
- ceres-solver: `v1.13.0`

### protobuf
**note**: do not execute `src/cartographer/scripts/install_proto3.sh` which is recommended in [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html). Please follow the [this instruction](https://gist.github.com/tongtybj/c5e0cec0160c2194298ee4b5895c3753). 

## Dataset: [URL1](https://drive.google.com/drive/folders/1iuQvsW0FCaBXpoltxpyVablFweW_NsLd), [URL2](https://drive.google.com/open?id=1F3-dNg1OLLCZD30C0u_Kt1BXGl_AhBsT)

## Usage with PR2

### In Gazebo

#### 1. bringup pr2 in gazebo
```
$ roslaunch cartographer_ros_pr2 pr2_simulation.launch
```
#### 2. start laser tiling control
```
$ rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: "linear" , period: 3 , amplitude: 1 , offset: 0 }}'
```
#### 3. start teleop
```
$ roslaunch pr2_teleop teleop_keyboard.launch
```

#### 4(a). start 2D cartgrapher
```
$ roslaunch cartographer_ros_pr2 cartographer_pr2_2d.launch simulation:=true
```

#### 4(b). start 3D cartgrapher
```
$ roslaunch cartographer_ros_pr2 cartographer_pr2_3d.launch simulation:=true
```

### In realmachine

#### online slam
```
$ roslaunch cartographer_ros_pr2 cartographer_pr2_3d.launch
```
- TODO: cartographer conflicts with amcl

#### offline slam
**note**: download [2019-03-01-pr2-eng2-7th.bag](https://drive.google.com/open?id=1-LhhKQPMewhWWOiqEeyQKjcUvF2lkUQu)
```
$ roslaunch cartographer_ros_pr2  cartographer_pr2_3d.launch offline:=true bag_filename:=${HOME}/Downloads/2019-03-01-pr2-eng2-7th.bag map_offset_x:=5 map_offset_y:=5.5 map_offset_yaw:=2.35
```

## Usage with sensor units

### case1: velodyne HDL-32e + original IMU board (spinal) with hand movement:
#### command for online slam
- note1: connect velodyne HDL-32e to host PC via Ethernet cable (following [wiliki](http://www.jsk.t.u-tokyo.ac.jp/wiliki/wiliki.cgi?Velodyne_Laser_Setting))
- note2: spinal IMU board to host PC via USB cable (do imu calibration if necessary)

```
$ roslaunch velodyne_spinal_hand.launch headless:=false
```

#### command for sensor data recording

- note1: connect velodyne HDL-32e to host PC via Ethernet cable (following [wiliki](http://www.jsk.t.u-tokyo.ac.jp/wiliki/wiliki.cgi?Velodyne_Laser_Setting))
- note2: spinal IMU board to host PC via USB cable (do imu calibration if necessary)

```
$ roslaunch cartographer_ros_pr2 velodyne_spinal_hand.launch record:=true
$ rosrun cartographer_ros_pr2 velodyne_spinal_hand_record.sh
```

#### command for offline slam using rosbag
**note**: download [2019-03-01-velodyne_imu_usb_cam_eng8-2-3.bag](https://drive.google.com/open?id=1VUbnJ_ThCOZqkMFWXVQaeDXbSkyE0ZZD)
```
$ roslaunch cartographer_ros_pr2 velodyne_spinal_hand.launch offline:=true
$ rosbag play ${HOME}/Downloads/2019-03-01-velodyne_imu_usb_cam_eng8-2-3.bag --clock
```

#### pure localization (set initial pose from rviz, please check [PR #1284](https://github.com/googlecartographer/cartographer_ros/pull/1284))

1. run the pure localization mode:

**note**: download [velodyne-imu-eng8-2-3.pbstream](https://drive.google.com/open?id=1mPGdI8nq-nxTepCWD_NEAEHXTB97ENXp)
```
 $ roslaunch cartographer_ros_pr2 velodyne_spinal_pure_localization.launch load_state_filename:=${HOME}/Downloads/velodyne-imu-eng8-2-3.pbstream
```

2. set the initial pose from rviz using icon `2D Pose Estimate`

3. play a rosbag (download [2019-03-01-velodyne_imu_usb_cam_eng2-8.bag](https://drive.google.com/open?id=1POLUDcSHjsPxg8YRwZgCt8WuiFKMkx98))
```
 $ rosbag play ${HOME}/Downloads/2019-03-01-velodyne_imu_usb_cam_eng2-8.bag --clock
```

### case2: ZED mini (stereo camera) with hand movement:
#### command for online slam
```
$ roslaunch cartographer_ros_pr2 zed_stereo.launch
```

## Usage with turtlebot

### 2D mode (kinect ) in gazebo

#### slam + navigation

```
$ roslaunch cartographer_ros_pr2 turtlebot_depth_camera_2d_gazebo.launch
```

**note**: you can also use "2D Nav Goal" icon in Rviz to perform navigation in gazebo world.

#### localization mode + navigation (navFN global path planner)

**note1**: you have to first create the .pbstream (map) file from aforementioned slam mode, or you can download [turtlebot_gazebo.pbstream](https://drive.google.com/open?id=1hABP6CYYyfUi67tcLEtS_j0XXljleJ5s), [turtlebot_gazebo2.pbstream](https://drive.google.com/open?id=1ahKHNuF4H2wzDMrXKOdIASHE8v07XEch)

```
$ roslaunch cartographer_ros_pr2 turtlebot_depth_camera_2d_gazebo.launch localization_mode:=true load_state_filename:=${HOME}/Downloads/turtlebot_gazebo2.pbstream
```

**note2**:  use "2D Pose Estimate" icon in Rviz to set the initial 2D pos for robot, and can also use "2D Nav Goal" icon in Rviz to perform navigation

#### localization mode + navigation (based on existing mapping path from cartographer)

**note1**: please download [turtlebot_gazebo3.pbstream](https://drive.google.com/open?id=1bPeZr5thyy-JaK9bf8Nj9I7TSuwR2LSf)

```
$ roslaunch cartographer_ros_pr2 turtlebot_depth_camera_2d_gazebo.launch localization_mode:=true existing_path_planning:=true load_state_filename:=${HOME}/Downloads/turtlebot_gazebo3.pbstream world_file:=/home/chou/ros/test_ws/src/test/cartographer_ros_pr2/worlds/turtlebot_playground.world
```

**note2**: use "2D Nav Goal" icon in Rviz to perform navigation as shown in [this image](https://drive.google.com/file/d/1He5qJICMJGG4p75X2nsNvr5DkP82z5Xb/view?usp=sharing)

**note3**: please carefully read [dwa_local_planner](http://wiki.ros.org/dwa_local_planner) and [costmap](http://wiki.ros.org/costmap_2d?distro=melodic) to understand the local path planning for [obstacle avoidance](https://github.com/tongtybj/test/blob/master/cartographer_ros_pr2/launch/turtlebot_depth_camera_2d_gazebo.launch#L106-L113).



### 3D mode (velodyne + spinal) in real machine
#### slam + navigation

```
$ roslaunch cartographer_ros_pr2 turtlebot_velodyne_spinal.launch
```

**note**: you can also use "2D Nav Goal" icon in Rviz to perform navigation in gazebo world. **However**, please stop the teleop (`roslaunch turtlebot_teleop keyboard_teleop.launch`)

#### localization mode + navigation (navFN global path planner)

**note1**: you have to first create the .pbstream (map) file from aforementioned slam mode, or you can download [eng8-3F.pbstream](https://drive.google.com/open?id=1R-9MXOzTxTEnQLmdoOTsgRyxwW-aDZZ-)

```
$ roslaunch cartographer_ros_pr2 turtlebot_velodyne_spinal.launch localization_mode:=true load_state_filename:=${HOME}/Downloads/eng8-3F.pbstream
```

**note2**:  use "2D Pose Estimate" icon in Rviz to set the initial 2D pos for robot, and can also use "2D Nav Goal" icon in Rviz to perform navigation


#### localization mode + navigation (based on existing mapping path from cartographer)

- host robot: 
  ```
  $ roslaunch cartographer_ros_pr2 turtlebot_velodyne_spinal.launch localization_mode:=true load_state_filename:=${HOME}/Downloads/velodyne-imu-eng8-2-3.pbstream existing_path_planning:=true
  ```

- local pc:
  ```
  $ rviz  -d  `rospack find cartographer_ros_pr2`/configuration_files/turtlebot_
  velodyne.rviz
  ```
  **note**:  use "2D Nav Goal" icon in remote Rviz to perform navigation
