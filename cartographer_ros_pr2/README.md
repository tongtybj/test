### About this package
Test of SLAM based on [Cartographer](https://github.com/googlecartographer) for PR2

### Install cartgrapher
Obtain the latest cartographer from [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)

### Usage in gazebo

#### 1. bringup pr2 in gazebo
```
$ roslaunch pr2_gazebo pr2_wg_world.launch
```
#### 2. start laser tiling control
```
$ rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: "linear" , period: 3 , amplitude: 1 , offset: 0 }}'
```
#### 3. start teleop
```
$ roslaunch pr2_teleop teleop_keyboard.launch
```

### 4(a). start 2D cartgrapher
```
$ roslaunch cartographer_ros_pr2 cartographer_pr2_2d.launch
```

### 4(b). start 3D cartgrapher
```
$ roslaunch cartographer_ros_pr2 cartographer_pr2_3d.launch
```

### Uasage for sensor units

#### case1: velodyne HDL-32e + original IMU board (spinal) with hand movement:
##### preparation1: connect velodyne HDL-32e to host PC via Ethernet cable (following [wiliki](http://www.jsk.t.u-tokyo.ac.jp/wiliki/wiliki.cgi?Velodyne_Laser_Setting))
##### preparation2: connect spinal IMU board to host PC via USB cable (do imu calibration if necessary)
##### command:
```
$ roslaunch velodyne_spinal_hand.launch headless:=false
```