# ROS code structure {#ros-structure status=ready}

Maintainer: Russell Buchanan

This document goes over the file structure in ROS projects.

[talker-py]: https://github.com/duckietown/Software/blob/master18/catkin_ws/src/60-templates/pkg_name/src/talker.py
[CMakeLists-txt]: https://github.com/duckietown/Software/blob/master18/catkin_ws/src/60-templates/pkg_name/CMakeLists.txt
[setup-py]: https://github.com/duckietown/Software/blob/master18/catkin_ws/src/60-templates/pkg_name/setup.py
[package-xml]: https://github.com/duckietown/Software/blob/master18/catkin_ws/src/60-templates/pkg_name/package.xml
[util-py]: https://github.com/duckietown/Software/blob/master18/catkin_ws/src/60-templates/pkg_name/include/pkg_name/util.py

To follow along, it is recommend that you duplicate the `pkg_name` folder and edit the content of the files to make your own package.

## Workspaces
Software development in ROS takes place in a root directory call a catkin workspace, usually named `catkin_ws`. Inside this directory is a `src` folder which contains all the packages. ROS is built be several packages which each perform a specific role such as defining rosnodes, actions or message definitions.
```
+-- catkin_ws
|   +-- build (auto generated)
|   +-- devel (auto generated)
|	+-- logs (auto generated)
|	+-- src
|		+-- my-ros-pkg
|			+-- package.xml
|			+-- CMakeLists.txt
|			+-- src
|     +--include
|       +--my-ros-pkg
|		+-- my-other-ros-pkg
|			+-- package.xml
|			+-- CMakeLists.txt
|			+-- source-files
|     +--include
|       +--my-other-ros-pkg
```
See the [pkg_name](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/60-templates/pkg_name) template as an example of a ROS package. It can be launched independently with:

	duckiebot $ roslaunch pkg_name talker.launch

The package.xml and CMakeLists.txt files are used for building the workspace and identifying all the dependancies.

## `CMakeLists.txt`

Take a look at the [`CMakeLists.txt`][CMakeLists-txt] file. Every ROS package needs a file `CMakeLists.txt`, even if you are just using Python code in your package. Read more about CMake [here](https://cmake.org/cmake-tutorial/).

For a Python package, we only care about the following parts:

This defines the name of the project:

    project(pkg_name)

This is a list of all the packages on which your package is dependent:

    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      duckietown_msgs # Every duckietown packages must use this.
      std_msgs
    )

In Duckietown, most packages depend on `duckietown_msgs` to make use of the customized messages.

We also care about this line:

    catkin_python_setup()

which tells `catkin` to setup Python related stuff for our package. Read more about `setup.py` [here](http://docs.ros.org/api/catkin/html/user_guide/setup_dot_py.html).


## `package.xml`

The file [`package.xml`][package-xml] defines the meta-data of the package. Catkin makes use of it to flush out the dependency tree and figures out the order of compiling.

Pay attention to the following parts:

`<name>` defines the name of the package. It must match the project name in `CMakeLists.txt`.

`<description>` describes the package concisely.

`<maintainer>` names the maintainer.

`<build_depend>` and `<run_depend>`. The catkin packages this package depends on. This usually match the `find_package` in `CMakeLists.txt`.


## `setup.py`

The file [`setup.py`][setup-py] configures the Python modules in this package.

The part to pay attention to is

    setup_args = generate_distutils_setup(
        packages=['pkg_name'],
        package_dir={'': 'include'},
    )

The `packages` parameter is set to a list of strings of the name of the folders inside the `include` folder. The convention is to set the folder name the same as the package name. Here it's the `include/pkg_name` folder. You should put ROS-independent and/or reusable module (for other packages) in the `include/pkg_name` folder.

Python files in this folder (for example, the `util.py`) will be available to scripts in the `catkin` workspace (this package and other packages too). To use these modules from other packages, use: `from pkg_name.util import *`