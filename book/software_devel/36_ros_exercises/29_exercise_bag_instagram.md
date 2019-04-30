# Exercise: Bag instagram {#exercise-bag-mirror status=ready}



## Instructions

Create `dt-bag-instagram` as specified below.


## Specification for `dt-bag-instagram`

Write a program `dt-bag-instagram` that applies a filter to a stream of images
stored in a ROS bag.

The syntax to invoke the program is:

    $ dt-bag-instagram ![bag in] ![topic] ![filters] ![bag out]

where:

    - `![bag in]` is the input bag;
    - `![topic]` is a string containing the topic to process;
    - `![filters]` is a string, which is a colon-separated list of filters;
    - `![bag out]` is the output bag.


## Test data

If you don't have a ROS bag to work on, you can download the test bag
[`example_rosbag_H5.bag`](https://www.dropbox.com/s/h04435fz8wv4314/example_rosbag_H5.bag?dl=1).


## Useful new APIs

### Compress an BGR image into a `sensor_msgs/CompressedImage` message

The [`duckietown_utils`](+software_devel#duckietown-utils-library)
package provides the utility function
[`d8_compressed_image_from_cv_image()`](+software_devel#duckietown_utils-d8_compressed_image_from_cv_image)
that takes a BGR image, compresses it and wraps it into a `sensor_msgs/CompressedImage`
ROS message.


## Check that it works

Play your `![bag out]` ROS bag file and run the following command to make sure
that your program is working.

    $ rosrun image_view image_view image:=![topic] _image_transport:=compressed
