can_plugins 
====
[![Build Status](https://travis-ci.org/ryugirou/robot.svg?branch=master)](https://travis-ci.org/ryugirou/robot)
## Usage
launchファイルでの使い方
```xml
  <arg name="manager_name" default="nodelet_manager" />
  <arg name="nodelet_mode" default="load" /><!-- set to standalone if you want to use as node-->
   
  <group if="$(eval nodelet_mode=='load')">
    <node pkg="nodelet" type="nodelet" name="$(arg manager_name)" args="manager" output="screen"/>
  </group>

  <!-- CAN -->
  <node pkg="nodelet" type="nodelet" name="usb_can_node" args="$(arg nodelet_mode) can_plugins/UsbCanNode $(arg manager_name)" output="screen">
    <param name="port" value="/dev/ttyUSB0"/>
  </node>    

  <node pkg="nodelet" type="nodelet" name="can_handler" args="$(arg nodelet_mode) can_plugins/CanHandler $(arg manager_name)" output="screen">
    <param name="beta0" value="$(eval 0x4e0)"/>
    <param name="beta1" value="$(eval 0x4b8)"/>
    <param name="beta2" value="$(eval 0x4d4)"/>
    <param name="beta3" value="$(eval 0x4d0)"/>
    <remap from="beta/motor0_cmd_vel" to="base/motor0_cmd_vel"/>
    <remap from="beta/motor1_cmd_vel" to="base/motor1_cmd_vel"/>
    <remap from="beta/motor2_cmd_vel" to="base/motor2_cmd_vel"/>
    <remap from="beta/motor3_cmd_vel" to="base/motor3_cmd_vel"/>
  </node>
```

## Install
udevファイルをコピーする
```
sudo cp ~/catkin_ws/src/robot/can_plugins/udev/60-usbcan.rules /etc/udev/rules.d/60-usbcan.rules
sudo udevadm control --reload-rules && udevadm trigger
```
