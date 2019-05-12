## About this package
Test of SLAM based on [Cartographer](https://github.com/googlecartographer) for PR2

## Install cartgrapher
Obtain the latest cartographer from [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html).


### version requirement
- cartographer: `master` (**note**: please checkout to `master` branch (not `v1.0.0`))
- cartographer_ros: [`tongtybj:set_initpose_from_rviz`](https://github.com/tongtybj/cartographer_ros/tree/set_initpose_from_rviz)  (opening PR [#1284](https://github.com/googlecartographer/cartographer_ros/pull/1284))
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

3. play a rosbag
```
 $ rosbag play ${HOME}/Downloads/2019-03-01-velodyne_imu_usb_cam_eng2-8.bag --clock
```

### case2: ZED mini (stereo camera) with hand movement:
#### command for online slam
```
$ roslaunch cartographer_ros_pr2 zed_stereo.launch
```
