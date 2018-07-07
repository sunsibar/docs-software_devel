# Configuration {#configuration-files status=beta}

This chapter explains what are the assumptions about the configuration.

While the "Setup" parts are "imperative" (do this, do that); this is the
"declarative" part, which explains what are the properties of a correct
configuration (but it does not explain how to get there).

The tool `what-the-duck` checks some of these conditions.
If you make a change from the existing conditions, make sure that it gets
implemented in `what-the-duck` by filing an issue.

## Environment variables (updated Sept 12) {#env-variables}

You need to have set up the variables in [](#tab:environment-variables).

Note: The way to set these up is to add them in the file `~/.bashrc` (`export ![var]="![value]"`). Do not modify the `environment.sh` script.


<col3 figure-id="tab:environment-variables" class='labels-row1'>
   <figcaption>Environment variables used by the software</figcaption>
    <s>variable</s>
    <s>reasonable value</s>
    <s>contains</s>
    <s><code>DUCKIETOWN_ROOT</code></s>
    <s><code>~/duckietown</code></s>
    <s><code>Software</code> repository</s>
    <s><code>DUCKIEFLEET_ROOT</code></s>
    <s><code>~/duckiefleet</code></s>
    <s>Where to look for class-specific information (people DB, robots DB).</s>
    <s><code>DUCKIETOWN_DATA</code></s>
    <s><code>~/duckietown-data</code></s>
    <s>The place where to look for logs.</s>
    <s><code>DUCKIETOWN_TMP</code></s>
    <s></s>
    <s>If set, directory to use for temporary files. If not set, we use the default ( <code>/tmp</code>).</s>
    <s><code>DUCKIETOWN_CONFIG_SEQUENCE</code></s>
    <s><code>defaults:baseline:vehicle:user</code></s>
    <s>The <a href="+code_docs#easy_node">configuration sequence for EasyNode</a></s>
</col3>

<style>
#tab\:environment-variables {
    font-size: 80%;
}
#tab\:environment-variables td {
    text-align: left;
}
#tab\:environment-variables td:nth-child(3) {
    display: block;
    width: 20em;
}
</style>

### Duckietown root directory `DUCKIETOWN_ROOT` {#duckietown-directory status=draft}

TODO: to write

### Duckiefleet directory `DUCKIEFLEET_ROOT` {#duckiefleet-directory}

For Fall 2017, this is the the repository [`duckiefleet`][duckiefleet-repo].

For self-guided learners, this is an arbitrary repository to create.

[duckiefleet-repo]: https://github.com/duckietown/duckiefleet


## The "scuderia" (vehicle database) {#scuderia}
<!-- do not change the ID "scuderia", it's linked in the code -->

The system needs to know certain details about the robots, such as their host names,
the name of the users, etc.

This data is contained in the <code>&#36;{DUCKIEFLEET_ROOT}/robots/{your_branch}</code> directory,
in files with the pattern `![robot name].robot.yaml`.

The file must contain YAML entries of the type:

    owner: ![ID of owner]
    username: ![username on the machine]
    hostname: ![host name]
    description: ![generic description]
    log:
        ![date]: ![comment]
        ![date]: ![comment]

A minimal example is in [](#code:scuderia-minimal).

<div figure-id="code:scuderia-minimal" markdown="1">
<figcaption>Minimal scuderia file <code>emma.robot.yaml</code></figcaption>
```yaml
owner: censi
hostname: emma
username: andrea
description: Andrea's car.
log:
    2017-08-01: >
        Switched RPI2 with RPI3.
    2017-08-20: >
        There is something wrong with the PWM hat and the LEDs.
```
</div>

Explanations of the fields:

* `![hostname]`: the host name. This is normally the same as the robot name.
* `![username]`: the name of the Linux user on the robot, from which to run programs.
* `![owner]`: the owner's globally-unique Duckietown ID.


## The `machines` file {#machines}

<!-- do not change the ID "machines"; it's linked in the code -->

Make sure you already set up ROS ([](+opmanual_duckiebot#build-repo)).

Activate ROS:

    $ cd ~/duckietown
    $ source environment.sh

The `machines` file is created from the scuderia data using this command:

    $ rosrun duckieteam create-machines

## People database {#people-file status=draft}

Assigned: Andrea Censi


TODO: Describe the people database.


### The globally-unique Duckietown ID {status=outdated}

This is a globally-unique ID for people in the Duckietown project.

It is equal to the Slack username.

Comment: There is no Slack username anymore, so we should change this to some other convention. -AC


## Modes of operation {#modes-of-operation status=draft}

There are 3 modes of operation:

1. `MODE-normal`: Everything runs on the robot.
2. `MODE-offload`: Drivers run on the robot, but heavy computation runs on the laptop.
3. `MODE-bag`: The data is provided from a bag file, and computation runs on the laptop.

<!-- 4. `MODE-unittest`: This is the case where many unit tests are run in parallel, on the cloud.  -->

<col4 class='labels-row1' id='operation-modes' figure-id="tab:operation-modes" figure-caption="Operation modes">

    <s>mode name</s>
    <s>who is the ROS master</s>
    <s>where data comes from</s>
    <s>where heavy computation happen</s>

    <code>MODE-normal</code>
    <code>duckiebot</code>
    <s>Drivers on Duckiebot</s>
    <code>duckiebot</code>

    <code>MODE-offload</code>
    <code>duckiebot</code>
    <s>Drivers on Duckiebot</s>
    <code>laptop</code>

    <code>MODE-bag</code>
    <code>laptop</code>
    <s>Bag file</s>
    <code>laptop</code>
</col4>

<style>
#operation-modes {
font-size: 70%;
}
</style>
