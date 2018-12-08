# range_sensor_fusion

This repo is a rosnode that subscribes distance_sensor data from rostopic:
- /distance_forward
- /distance_middle
- /distance_backward

then fuse it and publish the real distance in the rostopic
- /mavros/distance_sensor/laser_1_sub

## Usage:

In your catkin workspace:

`git clone https://github.com/casy-px4/rosnode_distance_sensor.git`

Then

`catkin_make`

Finally

`cd devel && source setup.bash`

Now you can start it launching

`rosrun rosnode_distance_sensor rosnode_distance_sensor`