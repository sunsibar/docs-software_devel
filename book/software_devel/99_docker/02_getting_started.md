# Duckiebot Development using Docker {status=ready}


<style> 
figure img {
max-width: 100%;
}
</style>


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
* Browser and/or [Docker CE](https://www.docker.com/community-edition#/download)

## Installation

* Software Prerequisites (Ubuntu/Debian):

    * `wget`
    * `apt-get`
    * Docker (See [](#docker-intro) for installation instructions)

Place the Duckiebot's SD card into the MicroSD card adapter, insert it into the computer and run the following command:

    $ bash -c "$(wget -O- h.ndan.co)"

The above command runs the [`flash-hypriot.sh` script](https://raw.githubusercontent.com/rusi/duckietown.dev.land/master/assets/flash-hypriot.sh).

This will download and run an installer to prepare the SD card.

Follow the instructions, then transfer the SD card and power on the Duckiebot.

Wait for a minute, then visit the following URL:

`http://![DUCKIEBOT_NAME].local:9000/`

You should be greeted by the Portainer web interface.

This user-friendly web interface is the primary mechanism for interacting with a Duckiebot.

Here you can see the list of running containers on your machine:

<figure>
 <img src="pics/portainer.png"/>
 <figcaption>Portainer Container View</figcaption>
</figure>

You can attach a console to a running container and interact with it via the browser:

<figure>
 <img src="pics/portainer_duckieshell.png"/>
 <figcaption>Portainer Web Interface</figcaption>
</figure>


If you prefer the command line, you can also connect to the Duckiebot via secure shell:

    $ ssh ![USER_NAME]@![DUCKIEBOT_NAME].local

## Changing WIFI Access Point and PSK

Insert the Duckiebot's Micro SD card into the adapter and insert it into the computer.

The drive should be mounted under `/media/![USER]/root`.

Open the `/media/![USER]/root/etc/wpa_supplicant/wpa_supplicant.conf` file and update the `ssid` and `psk` fields.

Save the file and unmount the drive, then insert the SD card into the Raspberry Pi and power on the Duckiebot.

## Testing the camera 

Open Portainer Web interface and run the `duckietown/rpi-docker-python-picamera` container. 

Publish port 8080 and ensure that the container is run in "Privileged" mode.

<figure>
 <img src="pics/picam-container.png"/>
 <figcaption>Portainer PiCam Demo</figcaption>
</figure>


Alternatively, run via the command line:


    $ docker -H ![DUCKIEBOT_NAME].local run -d --name picam \
      -v /data:/data \
      --privileged -p 8080:8080 duckietown/rpi-docker-python-picamera

Note: The syntax `-H ![DUCKIEBOT_NAME].local` may be omitted if you are running the command over SSH.

Note: adding the `-v /data:/data` option would persist the captured image on the Duckiebot's SD card.

Go to the following URL: `http://![DUCKIEBOT_NAME].local:8080/image.jpg`

## Running Simple HTTP File Server

All persistent data is stored under `/data` on the Duckiebot SD card. To access the data via the web browser, run:

    $ docker -H ![DUCKEBOT_NAME].local run -d --name file-server \
      -v /data:/data \
      -p 8080:8080 duckietown/rpi-simple-server

Go to the following URL: `http://![DUCKIEBOT_NAME].local:8080/`

Note: make sure that the `picam` container is stopped. Alternative, you can use a different port:

    $ docker -H ![DUCKIEBOT_NAME].local run -d --name file-server \
    -v /data:/data \
    -p 8080:8090 duckietown/rpi-simple-server

TODO: Utilize external storage device in keeping with [configuration DB17-d](+opmanual_duckiebot#duckiebot-configurations).

## Testing ROS

It is best to first pull the `base` Duckietown Docker image using the following command:

    $ docker -H ![DUCKIEBOT_NAME].local pull duckietown/rpi-ros-kinetic-roscore


Run the `base` Duckietown Docker image, opening a shell:


    $ docker -H ![DUCKIEBOT_NAME].local run -it --name roscore \
      --privileged \
      --net host \
      duckietown/rpi-ros-kinetic-roscore


You can start a ROS environment on your laptop, which connects to the Duckiebot ROS Master:

```
$ nvidia-docker run -it --rm --name ros \
  --net host \
  --env ROS_HOSTNAME=$HOSTNAME \
  --env ROS_MASTER_URI=http://![DUCKIEBOT_IP]:11311 \
  --env ROS_IP=![LAPTOP_IP] \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"\
  rosindustrial/ros-robot-nvidia:kinetic
```

To allow incoming X connections, run `xhost +` on your computer. 

Note: There is a [more secure way](http://wiki.ros.org/docker/Tutorials/GUI#The_safer_way) to do this, if you are concerned about receiving arbitrary X11 connections.

The above command opens a "ROS" shell running on your laptop that is set to connect to `DUCKIEBOT`'s ROS Master.
To test the ROS connection, run `roswtf`:

    $ roswtf

----------------------

## Test ROS Joystick

```
$ docker -H ![DUCKIEBOT].local run -d --name joystick-demo \
    --privileged \
    -v /data:/data \
    --net host \
    duckietown/rpi-duckiebot-joystick-demo
```

## Calibration

As described in [](+opmanual_duckiebot#camera-calib), print the calibration pattern and place the Duckiebot in the proper position.

### Extrinsic calibration procedure

Launch the calibration container and follow the prompts:

```
$ docker -H ![DUCKIEBOT_NAME].local run -it --name calibration \
  --privileged \
  -v /data:/data \
  --net host \
  duckietown/rpi-duckiebot-calibration
```

You will first be asked to place the Duckiebot on the calibration pattern. Then you will be asked to place in a lane to test the calibration.

Note: Passing `-v /data:/data` is necessary so that all calibration settings will be preserved.

Note: You can run/launch the `rpi-simple-server` to see the results in your web browser; you can also download all files from `/data`. This is an easy way to view and download all calibration files and validation results.

## Lane Following Demo

After the Duckiebot has been calibrated, you can now launch the [Lane Following Demo](+opmanual_duckiebot#demo-lane-following).


    $ docker -H ![DUCKIEBOT_NAME].local run -it --name lanefollowing-demo \
      --privileged \
      -v /data:/data \
      --net host \
      duckietown/rpi-duckiebot-lanefollowing-demo


Wait for a few minutes for all nodes to be started and initialized.

You can test the Duckiebot by using the Joystick. Pressing `R1` starts `autonomous` mode.

Pressing `L1` puts the Duckiebot back in `manual` mode.


## Docker Image Hierarchy


<figure>
 <img src="pics/docker_diagram.svg"/>
 <figcaption>Docker Image Hierarchy</figcaption>
</figure>


## Resources and References

SC Card Configuration and Flashing script

* https://github.com/rusi/duckietown.dev.land/tree/master/assets
* https://raw.githubusercontent.com/rusi/duckietown.dev.land/master/assets/flash-hypriot.sh

RPi Camera Test container

* https://github.com/rusi/rpi-docker-python-picamera
* https://hub.docker.com/r/duckietown/rpi-docker-python-picamera/

RPi Simple HTTP File Server

* https://github.com/rusi/rpi-simple-server
* https://hub.docker.com/r/duckietown/rpi-simple-server/

Duckiebot ROS containers:

Base ROS container - base ROS container (does NOT have `picamera`); opens `bash` when launched

* https://github.com/duckietown/rpi-ros-kinetic-base
* https://hub.docker.com/r/duckietown/rpi-ros-kinetic-base

Base ROS container with development tools and Duckietown dependencies (includes `picamera`)

* https://hub.docker.com/r/duckietown/rpi-ros-kinetic-dev

`roscore` container - starts `roscore` when launched

* https://github.com/duckietown/rpi-ros-kinetic-roscore
* https://hub.docker.com/r/duckietown/rpi-ros-kinetic-roscore

Duckietown Base (monolithic) software container - opens `bash` when launched

* https://github.com/duckietown/Software
* https://hub.docker.com/r/duckietown/rpi-duckiebot-base

Joystick Demo container

* https://github.com/duckietown/rpi-duckiebot-joystick-demo
* https://hub.docker.com/r/duckietown/rpi-duckiebot-joystick-demo

Calibration container

* https://github.com/duckietown/rpi-duckiebot-calibration
* https://hub.docker.com/r/duckietown/rpi-duckiebot-calibration

Lane Following Demo container

* https://github.com/duckietown/rpi-duckiebot-lanefollowing-demo
* https://hub.docker.com/r/duckietown/rpi-duckiebot-lanefollowing-demo

Desktop ROS containers
* rosindustrial/ros-robot-nvidia:kinetic
    * https://github.com/ros-industrial/docker
    * https://hub.docker.com/r/rosindustrial/ros-robot-nvidia/
* osrf/ros:kinetic-desktop-full 
    * https://github.com/osrf/docker_images/blob/master/ros/kinetic/ubuntu/xenial/desktop-full/
    * https://hub.docker.com/r/osrf/ros/


## Misc

### Building images:

```
$ cd image-builder-rpi
$ docker build . --tag ![TAG_NAME]
```

### Transferring Docker containers

```
$ docker save ![TAG_NAME] | ssh -C duckie@![DUCKIEBOT_NAME].local docker load
```


<figure markdown="1">
 <img src="pics/rqt_dep_joystick.png"/>
 <figcaption> Output of `rqt_dep joystick` (compilation dependencies)</figcaption>
</figure>


<figure markdown="1">
 <img src="pics/rqt_graph_joystick.png"/>
 <figcaption> Output of `rqt_graph joystick` (runtime dependencies)</figcaption>
</figure> 
