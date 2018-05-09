# create2_ros
ROS Driver for iRobot Create2

## Controller Example


```
roslaunch create2_driver test_controller_with_odometry.launch
rostopic pub -r 10 /goal create2_controller/Goal '{x: 1.0, y: 0.0}'
```
