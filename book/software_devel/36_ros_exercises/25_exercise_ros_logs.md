# Exercise: Simple data analysis from a bag  {#exercise-bag-analysis status=beta}

Assigned: Andrea Daniele

## Skills learned

- Reading Bag files.
- Statistics functions (mean, median) in Numpy.
- Use YAML format.

## Instructions

Create an implementation of `dt-bag-analyze` according to the specification below.

<!-- Call the script `bag-analyze-![username]`. -->


## Specification for `dt-bag-analyze`

Create a program that summarizes the statistics of data in a bag file.

    $ dt-bag-analyze ![bag file]

Compute, for each topic in the bag:

* The total number of messages.
* The minimum, maximum, average, and median interval between successive messages, represented in seconds.

Print out the statistics using the YAML format.
Example output:

    $ dt-bag-analyze ![bag file]
    "![topic name]":
        num_messages: ![value]
        period:
            min: ![value]
            max: ![value]
            average: ![value]
            median: ![value]


## Useful APIs

### Read a ROS bag

A bag is a file format in ROS for storing ROS message data. The package `rosbag`
defines the class [`Bag`](http://docs.ros.org/api/rosbag/html/python/)
that provides all the methods needed to serialize messages to and from a single
file on disk using the bag format.

### Time in ROS

In ROS the time is stored as an object of type
[`rostime.Time`](http://docs.ros.org/diamondback/api/rospy/html/rospy.rostime.Time-class.html).
An object `t`, instance of
[`rostime.Time`](http://docs.ros.org/diamondback/api/rospy/html/rospy.rostime.Time-class.html),
represents a time instant as the number of
seconds since epoch (stored in `t.secs`) and the number of nanoseconds since
`t.secs` (stored in `t.nsecs`). The utility function
[`t.to_sec()`](http://docs.ros.org/diamondback/api/rospy/html/roslib.rostime.TVal-class.html#to_sec)
returns the time (in seconds) as a floating number.


## Test that it works

Download the ROS bag
[`example_rosbag_H3.bag`](https://www.dropbox.com/s/11t9p8efzjy1az9/example_rosbag_H3.bag?dl=1).
Run your program on it and compare the results:

    $ dt-bag-analyze example_rosbag.bag
    /tesla/camera_node/camera_info:
      num_messages: 653
      period:
        min: 0.01
        max: 0.05
        average: 0.03
        median: 0.03

    /tesla/line_detector_node/segment_list:
      num_messages: 198
      period:
        min: 0.08
        max: 0.17
        average: 0.11
        median: 0.1

    /tesla/wheels_driver_node/wheels_cmd:
      num_messages: 74
      period:
        min: 0.02
        max: 4.16
        average: 0.26
        median: 0.11
