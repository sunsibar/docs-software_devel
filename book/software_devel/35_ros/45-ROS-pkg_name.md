# Minimal ROS node - `pkg_name` {#ros-python-howto status=ready}



This document outline the process of writing a ROS package and nodes in Python.

[talker-py]: https://github.com/duckietown/Software/blob/master/catkin_ws/src/60-templates/pkg_name/src/talker.py
[CMakeLists-txt]: https://github.com/duckietown/Software/blob/master/catkin_ws/src/60-templates/pkg_name/CMakeLists.txt
[setup-py]: https://github.com/duckietown/Software/blob/master/catkin_ws/src/60-templates/pkg_name/setup.py
[package-xml]: https://github.com/duckietown/Software/blob/master/catkin_ws/src/60-templates/pkg_name/package.xml
[util-py]: https://github.com/duckietown/Software/blob/master/catkin_ws/src/60-templates/pkg_name/include/pkg_name/util.py

To follow along, it is recommend that you duplicate the `pkg_name` folder and edit the content of the files to make your own package.

## The files in the package

### `CMakeLists.txt`

We start with the file [`CMakeLists.txt`][CMakeLists-txt].

Every ROS package needs a file `CMakeLists.txt`, even if you are just using Python code in your package.

See also: documentation about CMakeLists.txt XXX

For a Python package, you only have to pay attention to the following parts.

The line:

    project(pkg_name)

defines the name of the project.

The `find_package` lines:


    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      duckietown_msgs # Every duckietown packages must use this.
      std_msgs
    )

You will have to specify the packages on which your package is dependent.

In Duckietown, most packages depend on `duckietown_msgs` to make use of the customized messages.

The line:

    catkin_python_setup()

tells `catkin` to setup Python-related stuff for this package.

See also: [ROS documentation about `setup.py`][setuppy]

[setuppy]: http://docs.ros.org/api/catkin/html/user_guide/setup_dot_py.html

### `package.xml`

The file [`package.xml`][package-xml] defines the meta data of the package.

Catkin makes use of it to flush out the dependency tree and figures out the order of compiling.

Pay attention to the following parts.

<code>&lt;name&gt;</code> defines the name of the package. It has to match the project name in `CMakeLists.txt`.

<code>&lt;description&gt;</code> describes the package concisely.

<code>&lt;maintainer&gt;</code> provides information of the maintainer.

<code>&lt;build_depend&gt;</code> and <code>&lt;run_depend&gt;</code>. The catkin packages this package depends on. This usually match the `find_package` in `CMakeLists.txt`.

### `setup.py`

The file [`setup.py`][setup-py] configures the Python modules in this package.

The part to pay attention to is

    setup_args = generate_distutils_setup(
        packages=['pkg_name'],
        package_dir={'': 'include'},
    )

The `packages` parameter is set to a list of strings of the name of the folders inside the `include` folder.

The convention is to set the folder name the same as the package name. Here it's the `include/pkg_name` folder.

You should put ROS-independent and/or reusable module (for other packages) in the `include/pkg_name` folder.

Python files in this folder (for example, the `util.py`) will be available to scripts in the `catkin` workspace (this package and other packages too).

To use these modules from other packages, use:

    from pkg_name.util import *


## Writing a node: `talker.py`

Let's look at [`src/talker.py`][talker-py] as an example.

ROS nodes are put under the `src` folder and they have to be made executable to function properly.

See: You use `chmod` for this; see [](+software_reference#chmod).

### Header

Header:

    #!/usr/bin/env python
    import rospy
    # Imports module. Not limited to modules in this package.
    from pkg_name.util import HelloGoodbye
    # Imports msg
    from std_msgs.msg import String

The first line, `#!/usr/bin/env python`, specifies that the script is written in Python.

Every ROS node in Python must start with this line.

The line `import rospy` imports the `rospy` module necessary for all ROS nodes in Python.

The line `from pkg_name.util import HelloGoodbye` imports the class `HelloGoodbye` defined in the file [`pkg_name/util.py`][util-py].

Note that you can also include modules provided by other packages, if you
specify the dependency in `CMakeLists.txt` and `package.xml`.

The line `from std_msgs.msg import String` imports the `String` message defined in the `std_msgs` package.

Note that you can use `rosmsg show std_msgs/String `
in a terminal to lookup the definition of `String.msg`.

### Main

This is the main file:

    if __name__ == '__main__':
        # Initialize the node with rospy
        rospy.init_node('talker', anonymous=False)

        # Create the NodeName object
        node = Talker()

        # Setup proper shutdown behavior
        rospy.on_shutdown(node.on_shutdown)

        # Keep it spinning to keep the node alive
        rospy.spin()


The line `rospy.init_node('talker', anonymous=False)` initializes a node named `talker`.

Note that this name can be overwritten by a launch file. The launch file can also push this node down namespaces. If the `anonymous` argument is set to `True` then a random string of numbers will be append to the name of the node. Usually we don't use anonymous nodes.

The line `node = Talker()` creates an instance of the Talker object. More details in the next section.

The line `rospy.on_shutdown(node.on_shutdown)` ensures that the `node.on_shutdown` will be called when the node is shutdown.

The line `rospy.spin()` blocks to keep the script alive. This makes sure the node stays alive and all the publication/subscriptions work correctly.

## The `Talker` class

We now discuss the `Talker` class in [`talker.py`][talker-py].

<!--
    class Talker(object):
        def __init__(self):
            # Save the name of the node
            self.node_name = rospy.get_name()

            rospy.loginfo("[%s] Initializing." % (self.node_name))

            # Setup publishers
            self.pub_topic_a = rospy.Publisher("~topic_a", String, queue_size=1)
            # Setup subscriber
            self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
            # Read parameters
            self.pub_timestep = setupParameter("~pub_timestep",1.0)
            # Create a timer that calls the cbTimer function every 1.0 second
            self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

            rospy.loginfo("[%s] Initialzed." %(self.node_name))

        def setupParameter(self, param_name, default_value):
            value = rospy.get_param(param_name,default_value)
            # Write to parameter server for transparancy
            rospy.set_param(param_name,value)
            rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
            return value

        def cbTopic(self,msg):
            rospy.loginfo("[%s] %s" %(self.node_name,msg.data))

        def cbTimer(self,event):
            singer = HelloGoodbye()
            # Simulate hearing something
            msg = String()
            msg.data = singer.sing("duckietown")
            self.pub_topic_name.publish(msg)

        def on_shutdown(self):
            rospy.loginfo("[%s] Shutting down." %(self.node_name))
 -->

### Constructor

In the constructor, we have:

    self.node_name = rospy.get_name()

saves the name of the node.

This allows to include the name of the node in printouts to make them more informative. For example:

    rospy.loginfo("[%s] Initializing." % (self.node_name))


The line:

    self.pub_topic_a = rospy.Publisher("~topic_a", String, queue_size=1)

defines a publisher which publishes a `String` message to the topic `~topic_a`. Note that the `~` in the name of topic under the namespace of the node. More specifically, this will actually publish to `talker/topic_a` instead of just `topic_a`. The `queue_size` is usually set to 1 on all publishers.

See: For more details see [`rospy` overview: publisher and subscribers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers).

The line:

    self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)

defines a subscriber which expects a `String` message and subscribes to `~topic_b`. The message will be handled by the `self.cbTopic` callback function. Note that similar to the publisher, the `~` in the topic name puts the topic under the namespace of the node. In this case the subscriber actually subscribes to the topic `talker/topic_b`.

It is strongly encouraged that a node always publishes and subscribes to topics under their `node_name` namespace. In other words, always put a `~` in front of the topic names when you defines a publisher or a subscriber. They can be easily remapped in a launch file. This makes the node more modular and minimizes the possibility of confusion and naming conflicts. See [the launch file section](#howto-launch-file) for how remapping works.

The line

    self.pub_timestep = self.setupParameter("~pub_timestep", 1.0)

Sets the value of `self.pub_timestep` to the value of the parameter `~pub_timestep`. If the parameter doesn't exist (not set in the launch file), then set it to the default value `1.0`. The `setupParameter` function also writes the final value to the parameter server. This means that you can `rosparam list` in a terminal to check the actual values of parameters being set.

The line:

    self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep), self.cbTimer)

defines a timer that calls the `self.cbTimer` function every `self.pub_timestep` seconds.

### Timer callback

Contents:

    def cbTimer(self,event):
        singer = HelloGoodbye()
        # Simulate hearing something
        msg = String()
        msg.data = singer.sing("duckietown")
        self.pub_topic_name.publish(msg)

Everyt ime the timer ticks, a message is generated and published.

### Subscriber callback

Contents:

    def cbTopic(self,msg):
        rospy.loginfo("[%s] %s" %(self.node_name,msg.data))


Every time a message is published to `~topic_b`, the `cbTopic` function is called. It simply prints the message using `rospy.loginfo`.

## Launch File {#howto-launch-file}

You should always write a launch file to launch a node. It also serves as a documentation on the I/O of the node.

Let's take a look at `launch/test.launch`.

    <launch>
        <node name="talker" pkg="pkg_name" type="talker.py" output="screen">
            <!-- Setup parameters -->
            <param name="~pub_timestep" value="0.5"/>
            <!-- Remapping topics -->
            <remap from="~topic_b" to="~topic_a"/>
        </node>
    </launch>

For the <code>&lt;node&gt;</code>, the `name` specify the name of the node,
which overwrites `rospy.init_node()` in the `__main__` of `talker.py`. The
`pkg` and `type` specify the package and the script of the node, in this case
it's `talker.py`.

Don't forget the `.py` in the end (and remember to make the file executable through `chmod`).

The `output="screen"` direct all the
`rospy.loginfo` to the screen, without this you won't see any printouts (useful
when you want to suppress a node that's too talkative.)

The <code>&lt;param&gt;</code> can be used to set the parameters. Here we set the `~pub_timestep` to `0.5`. Note that in this case this sets the value of `talker/pub_timestep` to `0.5`.

The <code>&lt;remap&gt;</code> is used to remap the topic names. In this case we are replacing `~topic_b` with `~topic_a` so that the subscriber of the node actually listens to its own publisher. Replace the line with

    <remap from="~topic_b" to="talker/topic_a"/>

will have the same effect. This is redundant in this case but very useful when you want to subscribe to a topic published by another node.

## Testing the node

First of all, you have to `catkin_make` the package even if it only uses Python. `catkin` makes sure that the modules in the include folder and the messages are available to the whole workspace. You can do so by

    $ cd ${DUCKIETOWN_ROOT}/catkin_ws
    $ catkin_make

Ask ROS to re-index the packages so that you can auto-complete most things.

    $ rospack profile

Now you can launch the node by the launch file.

    $ roslaunch pkg_name test.launch

You should see something like this in the terminal:

    ... logging to /home/![username]/.ros/log/d4db7c80-b272-11e5-8800-5c514fb7f0ed/roslaunch-![robot name]-15961.log
    Checking log directory for disk usage. This may take awhile.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is 1GB.

    started roslaunch server http://![robot name].local:33925/

    SUMMARY
    ========

    PARAMETERS
     * /rosdistro: $ROS_DISTRO
     * /rosversion: 1.11.16
     * /talker/pub_timestep: 0.5

    NODES
      /
        talker (pkg_name/talker.py)

    auto-starting new master
    process[master]: started with pid [15973]
    ROS_MASTER_URI=http://localhost:11311

    setting /run_id to d4db7c80-b272-11e5-8800-5c514fb7f0ed
    process[rosout-1]: started with pid [15986]
    started core service [/rosout]
    process[talker-2]: started with pid [15993]
    [INFO] [WallTime: 1451864197.775356] [/talker] Initialzing.
    [INFO] [WallTime: 1451864197.780158] [/talker] ~pub_timestep = 0.5
    [INFO] [WallTime: 1451864197.780616] [/talker] Initialzed.
    [INFO] [WallTime: 1451864198.281477] [/talker] Goodbye, duckietown.
    [INFO] [WallTime: 1451864198.781445] [/talker] Hello, duckietown.
    [INFO] [WallTime: 1451864199.281871] [/talker] Goodbye, duckietown.
    [INFO] [WallTime: 1451864199.781486] [/talker] Hello, duckietown.
    [INFO] [WallTime: 1451864200.281545] [/talker] Goodbye, duckietown.
    [INFO] [WallTime: 1451864200.781453] [/talker] Goodbye, duckietown.



Open another terminal and run:

    $ rostopic list

You should see

    /rosout
    /rosout_agg
    /talker/topic_a

In the same terminal, run:

    $ rosparam list

You should see the list of parameters, including `/talker/pub_timestep`.

You can see the parameters and the values of the `talker` node with

    $ rosparam get /talker

## Adding a command line parameter

You can register a parameter in the launch file such that it is added to the ROS parameter dictionary. This allows you to call `rospy.get_param()` on you parameter from `talker.py`.

Edit your launch file to look like this:

    <launch>
        <arg name="pub_timestep" default="0.5" />
        <node name="talker" pkg="pkg_name" type="talker.py" output="screen">
            <!-- Setup parameters -->
            <param name="~pub_timestep" value="$(arg pub_timestep)"/>
            <!-- Remapping topics -->
            <remap from="~topic_b" to="~topic_a"/>
        </node>
    </launch>

Previously, you should have had the line `<param name="~pub_timestep" value="0.5" />` inside of the node tags. This sets a parameter of value `0.5` to be called `/talker/pub_timestep`. (Remeber that the tilde prefixes the variable with the current namespace). By adding the line `<arg name="pub_timestep" default="1" />`, we are telling the program to look for a parameter on the command line called `pub_timestep`, and that if it doesn't find one, to use the value one. Then, <code>value=&#36;(arg pub_timestep)</code> retrieves the value set in the previous line.

Within `talker.py`, we can get the value of the inputted parameter. You should already have the line:

    self.pub_timestep = self.setupParameter("~pub_timestep", 1.0)

This calles the talker's `setupParameter` method, which contains the line:

    value = rospy.get_param(param_name,default_value)

Where `~pub_timestep` is passed in as `parapm_name` and `1.0` is passed in as the default value. Now that we have edited the launch file to accept a command line argument, `value` should be the value which is given on the command line, rather than `0.5`.

You can test that this works by calling `roslaunch` with the added parameter:

    $ roslaunch pkg_name test.launch pub_timestep:=3

This should cause the time between messages to become three seconds.

The functions `rosparam list` and `rosparam info [param]` are useful in debugging issues with registering a parameter.

## Documentation

You should document the parameters and the publish/subscribe topic names of each node in your package. The user should not have to look at the source code to figure out how to use the nodes.

## Guidelines

* Make sure to put all topics (publish or subscribe) and parameters under the namespace of the node with `~`. This makes sure that the IO of the node is crystal clear.
* Always include the name of the node in the printouts.
* Always provide a launch file that includes all the parameters (using <code>&lt;param&gt;</code>) and topics (using <code>&lt;remap&gt;</code>) with each node.
