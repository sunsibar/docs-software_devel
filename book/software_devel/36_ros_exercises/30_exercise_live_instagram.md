# Exercise: Live Instagram {#exercise-instagram-live status=ready}



## Skills learned

* Live image processing

## Instructions

You may find useful: [](+software_devel#ros-python-howto).
That tutorial is about listening to text messages and writing back
text messages. Here, we apply the same principle, but to images.

Create a ROS node that takes camera images and applies a given operation,
as specified in the next section, and then publishes it.


## Specification for the node `dt_live_instagram_![robot name]_node`

Create a ROS node `dt_live_instagram_![robot name]_node` that takes a parameter called `filter`, where the filter is something from the list [](#instagram-filters).

You should launch your camera and joystick from '~/duckietown' with

    duckiebot $ make demo-joystick-camera

Then launch your node with

    duckiebot $ roslaunch dt_live_instagram_![robot name] dt_live_instagram_![robot name]_node.launch filter:=![filter]

This program should do the following:

- Subscribe to the camera images, by finding
a topic that is called `![...]/compressed`. Call the name of the
topic `![topic]` (i.e., `![topic]` = `![...]`).

- Publish to the topic `![topic]/![filter]/compressed` a stream of images (i.e., video frames)
where the filter is applied to the images.


## Check that it works

    $ rqt_image_view

and look at `![topic]/![filter]/compressed`
