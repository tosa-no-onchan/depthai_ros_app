# depthai_ros_app
rgb_depth.cpp    
  
How to use.  

1. down load on Jetson Nano 2G  
$ cd ~catkin_ws/src  
$ git clone https://github.com/tosa-no-onchan/depthai_ros_app.git  
$ cd ..  
$ catkin_make --pkg depthai_ros_app -j1  
2. run  
on Remote PC  
$ roscore  
on Jeston Nano 2G  
$ ros run deptha_ros_app rgb_depth  
or  
$roslaunch deptha_ros_app rgb_depth.launch  
on Remote PC  
$rviz  
add  

