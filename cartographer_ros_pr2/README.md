## About this package
Test of SLAM based on [Cartographer](https://github.com/googlecartographer) for PR2

## Install cartgrapher
Obtain the latest cartographer from [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)

## Dataset URL: https://drive.google.com/drive/folders/1iuQvsW0FCaBXpoltxpyVablFweW_NsLd 

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
$ roslaunch cartographer_ros_pr2  cartographer_pr2_3d.launch offline:=true bag_filename:=${HOME}/2019-03-01-pr2-eng2-7th.bag map_offset_x:=5 map_offset_y:=5.5 map_offset_yaw:=2.35
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
$ roslaunch velodyne_spinal_hand.launch record:=true
$ rosrun cartographer_ros_pr2 velodyne_spinal_hand_d.sh
```

#### command for offline slam using rosbag
**note**: download [2019-03-01-velodyne_imu_usb_cam_eng8-2-3.bag](2019-03-01-velodyne_imu_usb_cam_eng8-2-3.bag)
```
$ roslaunch velodyne_spinal_hand.launch offline:=true
$ rosbag play ${HOME}/2019-03-01-velodyne_imu_usb_cam_eng8-2-3.bag --clock
```

#### pure localization
**note**: downlaod [2019-03-01-velodyne_imu_usb_cam_eng2-8.bag](https://drive.google.com/open?id=1POLUDcSHjsPxg8YRwZgCt8WuiFKMkx98) and [velodyne-imu-eng8-2-3.pbstream](https://drive.google.com/open?id=1mPGdI8nq-nxTepCWD_NEAEHXTB97ENXp)
```
 $ roslaunch cartographer_ros_pr2 velodyne_spinal_pure_localization.launch load_state_filename:=${HOME}/velodyne-imu-eng8-2-3.pbstream
 $ roslaunch velodyne_spinal_initial_pose.launch init_x:=-5.2 init_y:=-62.6 init_z:=-5 init_roll:=0.1 init_pitch:=0.1 init_yaw:=-0.2
 $ rosservice call /finish_trajectory "trajectory_id: 1"
 $ rosbag play ${HOME}/2019-03-01-velodyne_imu_usb_cam_eng2-8.bag --clock
```
