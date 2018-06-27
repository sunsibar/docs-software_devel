# Getting Started with Duckietown and Duckiebot: Autonomous Robotics, Reinforcement Learning and Raspberry Pi

## Prerequisites

Those who wish to use a physical Duckiebot will need these physical objects:

* Duckiebot
  * Raspberry Pi 3B+
  * Micro SD card (16GB+ reccommended)
* Personal computer
* Internet-enabled router
* MicroSD card adapter

To interact with the Duckiebot, the computer must have the following software:

* POSIX-compliant shell
* GNU wget
* Browser and/or Docker

## Installation

* Software Prerequisites (Ubuntu/Debian)
  * wget
  * apt-get
  * Docker

Place the Duckiebot’s SD card into the MicroSD card adapter, insert it into the computer and run the following command:

`sudo sh -c "$(wget -O- h.ndan.co)"`

This will download and run an installer to prepare the SD card.
Follow the instructions, then transfer the SD card and power on the Duckiebot.
Wait for a minute, then visit the following URL:

`http://<DUCKIEBOT_NAME>.local:9000/`

You should be greeted by a Portainer web interface.
This user-friendly web interface is the primary mechanism for interacting with a Duckiebot.
Here you can see the list of running containers on your machine:

![Portainer Container View](portainer.png)

You can attach a console to a running container and interact with it via the browser:

![Portainer Web Interface](portainer_duckieshell.png)

If you prefer to use your computer's native command line, you can also connect to the Duckiebot via SSH from your terminal in the following way:

`ssh <USER_NAME>@<DUCKIEBOT_NAME>.local`


# TODO:

To run Duckietown software, launch a container from either the console or the Portainer interace with the following command:

`docker run -it --privileged duckietown/software`

Depending on your connection speed, this step may take a while. Once inside the container, you may run some demos like so:

* `roslaunch pkg_name talker.launch`
* `roslaunch duckietown joystick.launch veh:=docker`
* `roslaunch duckietown_demos lane_following.launch line_detector_param_file_name:=$*`
