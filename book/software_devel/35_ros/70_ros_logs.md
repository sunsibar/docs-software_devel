# Using ROS bags {#ros-bags status=draft}

Maintainer: Russell Buchanan

***rosbags*** are a system for logging data in ROS. You can think of it as a node which subscribes to any topic you specify and records every message that is receives. rosbags can then play back the messages

## Commands

    $ rosbag record <topic names> -O bagname
    $ rosbag info
    $ rosbag play <bag file name>