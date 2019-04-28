# Exercise: Bag in, bag out {#exercise-bag-in-out status=ready}



## Skills learned

- Processing the contents of a bag to produce another bag.

## Instructions

Implement the program `dt-bag-decimate` as specified below.


## Specification of `dt-bag-decimate`

The program `dt-bag-decimate` takes as argument a bag filename, an integer
value greater than zero, and an output bag file:

    $ dt-bag-decimate "![input bag]" ![n] "![output bag]"

The output bag contains the same topics as the input bag, however, only 1 in
`n` messages from each topic are written.  (If `n` is 1, the output is the same as the input.)


## Useful new APIs

### Create a new Bag

In ROS, a new bag can be created by specifying the mode `w` (i.e., write) while
instantiating the class [`rosbag.Bag`][rosbag-bag].

For example:

    from rosbag import Bag
    new_bag = Bag('./output_bag.bag', mode='w')

Visit the documentation page for the class [`rosbag.Bag`][rosbag-bag] for further information.


[rosbag-bag]: http://docs.ros.org/api/rosbag/html/python/

### Write message to a Bag

A ROS bag instantiated in *write* mode accepts messages through the function
[`write()`](http://docs.ros.org/api/rosbag/html/python/rosbag.bag.Bag-class.html#write).


## Check that it works

To check that the program works, you can compute the statistics
of the data using the program `dt-bag-analyze` that you have created
in [](+exercises#exercise-bag-analysis).

You should see that the statistics have changed.
