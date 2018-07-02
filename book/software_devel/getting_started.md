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

`sh -c "$(wget -O- h.ndan.co)"`

The above command runs the `flash-hypriot.sh` script located at:
https://raw.githubusercontent.com/rusi/duckietown.dev.land/master/assets/flash-hypriot.sh

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

## Changing WIFI Access Point and PSK

Put the Duckiebot’s SD card into the MicroSD card adapter, insert it into the computer.
The drive should be mounted under `/media/<USER>/root`.
Edit the `/media/<USER>/root/etc/wpa_supplicant/wpa_supplicant.conf` file.
Update the `ssid` and `psk` fields.

Save the file and unmount the SD card.
Put the SD card into the Duckiebot and power on.

## Test PiCam

1. Open Portainer Web interface and run the "duckietown/rpi-docker-python-picamera" container; publish port 8080 and make sure that you run the container in "Privileged" mode.
![Portainer PiCam Demo](picam-container.png)
or run via command line:

`docker -H <DUCKIEBOT_NAME>.local run -d --name picam --privileged -p 8080:8080 duckietown/rpi-docker-python-picamera`

2. Go to the following URL: `http://<DUCKIEBOT_NAME>.local:8080/image.jpg`

## Test ROS

It is best to first pull the `base` Duckietown Docker image using the following command:

`docker -H <DUCKIEBOT_NAME>.local pull duckietown/rpi-ros-kinetic-roscore`

Run the `base` Duckietown Docker image, opening a shell:

`docker -H <DUCKIEBOT_NAME>.local run -it --rm --name duckieos --privileged --net host duckietown/rpi-ros-kinetic-roscore`

You can start a ROS environment on your laptop, that connects to the Duckiebot ROS Master:

`nvidia-docker run -it --rm --name ros --env ROS_HOSTNAME=$HOSTNAME --env ROS_MASTER_URI=http://<DUCKIEBOT_IP>:11311 --env ROS_IP=<LAPTOP_IP> --net host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  osrf/ros:kinetic-desktop-full`

The above command opens a "ROS" shell running on your laptop that is set to connect to <DUCKIEBOT>'s ROS Master.
To test the ROS connection, run `roswtf`:

`$ roswtf`

## Test ROS Joystick


## Resources & References

* SC Card Configuration & Flashing script
  * https://github.com/rusi/duckietown.dev.land/tree/master/assets
  * https://raw.githubusercontent.com/rusi/duckietown.dev.land/master/assets/flash-hypriot.sh

* RPi Camera Test container
  * https://github.com/rusi/rpi-docker-python-picamera
  * https://hub.docker.com/r/duckietown/rpi-docker-python-picamera/

* Duckiebot ROS containers
  * Base ROS container - opens bash shell when launched
    * https://github.com/duckietown/ducker/tree/master/rpi-ros-kinetic-base
    * https://hub.docker.com/r/duckietown/rpi-ros-kinetic-base
  * `roscore` container - starts `roscore` when launched
    * https://github.com/duckietown/ducker/tree/master/rpi-ros-kinetic-roscore
    * https://hub.docker.com/r/duckietown/rpi-ros-kinetic-roscore
  * Duckietown Base (monolithic) software container - opens bash shell when launched
    * https://github.com/duckietown/Software
    * https://hub.docker.com/r/duckietown/rpi-duckiesoft-base

* Joystick Demo container
  * https://github.com/duckietown/ducker/tree/master/rpi-duckiebot-joystick-demo
  * https://hub.docker.com/r/duckietown/rpi-duckiebot-joystick-demo

* Calibration container
  * https://github.com/duckietown/ducker/tree/master/rpi-duckiebot-calibration
  * https://hub.docker.com/r/duckietown/rpi-duckiebot-calibration

* Line Following Demo container
  * https://github.com/duckietown/ducker/tree/master/rpi-duckiebot-linefollowing-demo
  * https://hub.docker.com/r/duckietown/rpi-duckiebot-linefollowing-demo

* Desktop ROS containers
  * osrf/ros:kinetic-desktop-full

# TODO:

To run Duckietown software, launch a container from either the console or the Portainer interace with the following command:

`docker -H <DUCKIEBOT_NAME>.local run -it --privileged duckietown/software`

Depending on your connection speed, this step may take a while. Once inside the container, you may run some demos like so:

* `roslaunch pkg_name talker.launch`
* `roslaunch duckietown joystick.launch veh:=docker`
* `roslaunch duckietown_demos lane_following.launch line_detector_param_file_name:=$*`
modify lane_...
apt install libffi-dev
apt install libturbojpeg
install picamera - pip install
pip install jpeg4py
copy /home/duckiefleet/calibrations/camera_intrinsic/docker.yaml
copy /home/duckiefleet/calibrations/camera_extrinsic/docker.yaml
edit /home/software/catkin_ws/src/10-lane-control/lane_filter/include/lane_filter/lane_filter.py : 158
edit /home/software/catkin_ws/src/10-lane-control/lane_filter/src/lane_filter_node.py : 123


# Building images:

## duckietown/ducker.git

```
cd ducker/monolith
docker build . --tag monolith
```

## breandan/Software.git

```
cd Software
docker build . --tag duckieos
```

## Transferring Docker containers

```
docker save duckieos | ssh -C duckie@duckiebot.local docker load 
```

```
docker save duckieos | bzip2 | ssh duckie@duckiebot.local 'bunzip2 | docker load'
```

```
docker save duckieos | bzip2 | pv | ssh duckie@duckiebot.local 'bunzip2 | docker load'
```