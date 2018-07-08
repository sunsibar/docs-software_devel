# Duckietown logs facilities {#easy_logs status=ready}

Maintainer: Andrea Censi

This document describes how to use the tools for querying and downloading logs.

## Installation

### Option 1: Native

In the `Software` repo (branch `master18`) run the following:

    $ (cd catkin_ws/src/00-infrastructure/ && python setup.py develop --user )

TODO: add this in installation docs

### Option 2: Docker

You can run everything using Docker; see [](#dt-logs-containerized).

## Utilities summary

### `dt-logs-summary`: Querying the DB

You can query the cloud DB using the program `dt-logs-summary`:

    $  dt-logs-summary  

You can also include a query string: 

    $  dt-logs-summary ![query]

where `![query]` is a query expressed in the selector language ([](#selector-language)).

Here is an example output:

<pre>
 <code class='output'>
 &#36; dt-logs-summary "*dp3tele*"
 | #     Log name                          date          length    vehicle name    valid
 | --    ------------------------------    ----------    ------    ------------    -----------
 | 0     20160504-dp3tele1-thing           2016-05-04     294 s    thing           Yes.
 | 1     20160506-dp3tele1-nikola          2016-05-06     321 s    nikola          Yes.
 | 2     2016-04-29-dp3tele-neptunus-0     2016-04-29     119 s    neptunus        Yes.
 | 3     20160503-dp3tele1-pipquack        2016-05-03     352 s    pipquack        Yes.
 | 4     20160503-dp3tele1-redrover        2016-05-03     384 s    redrover        Yes.
 | 5     20160504-dp3tele1-penguin         None          (none)    None            Not indexed
 | 6     20160430-dp3tele-3-quackmobile    2016-04-30     119 s    quackmobile     Yes.
 </code>
</pre>

<style>
pre.output { font-size: 70%;}
</style>


### `dt-logs-download`: Downloading the logs

You can use the command `dt-logs-download` to download the logs:

    $ dt-logs-download 20160503164104_ayrton
             DT|    download.py:109  - download_url_to_file| Download from http://ipfs.duckietown.org:8080/ipfs/QmPrn3V1v6RkkwYf4eUM9UvnperNBt9ZG6JhGKm5a73Mgx using urllib
    ...0%, 0.01 MB of 918.2MB, 144.99 MB/s, 0 seconds passed
    ...14%, 129.55 MB of 918.2MB, 25.54 MB/s, 5 seconds passed
    ...28%, 263.76 MB of 918.2MB, 26.18 MB/s, 10 seconds passed
    ...42%, 391.12 MB of 918.2MB, 25.95 MB/s, 15 seconds passed
    ...56%, 520.31 MB of 918.2MB, 25.91 MB/s, 20 seconds passed
    ...70%, 649.75 MB of 918.2MB, 25.91 MB/s, 25 seconds passed
    ...85%, 781.15 MB of 918.2MB, 25.97 MB/s, 30 seconds passed
    ...98%, 900.80 MB of 918.2MB, 25.68 MB/s, 35 seconds passed
    ...100%, 918.17 MB of 918.2MB, 25.69 MB/s, 36 seconds passed
                 DT|    download.py:115  - download_url_to_file| -> ${DUCKIETOWN_TMP}/downloads/20160503-dp3tele1-ayrton.bag

### `dt-logs-find`: Get the local filename of a log

To get the location of the bag file, use the command `find`:

    $ dt-logs-find 20160503164104_ayrton
    /mnt/additional/duckietown-tmp/downloads/20160503-dp3tele1-ayrton.bag



A typical use case would be the following, in which a script needs a log with which to work.

Using the `dt-logs-download` program we declare that we need
the log. Then, we use `dt-logs-find` to find the path.

    #!/bin/bash
    set -ex

    # We need the log to proceed
    dt-logs-download 20160429223659_neptunus

    # Here, we know that we have the log. We use `find` to get the filename.
    filename=`dt-logs-find 20160429223659_neptunus`

    # We can now use the log
    vdir ${filename}
    
### `dt-logs-copy`: Copy the logs to a local directory

To copy the logs to a local directory, use the command `dt-logs-copy`:

    $ dt-logs-copy --outdir ![destination] 20160503164104_ayrton

### `dt-logs-details`: More details about the logs

Use `dt-logs-details` to have more details about a log:

    $ dt-logs-details 20160503164104_ayrton
    - - [log_name, dp3auto_2016-04-29-19-58-57_2-oreo]
      - [filename, ![...]]
      - [map_name, null]
      - [description, null]
      - [vehicle, oreo]
      - [date, '2016-04-29']
      - [length, 112.987947]
      - [size, 642088366]
      - - bag_info
        - compression: none
          duration: 112.987947
          end: 1461960050.639
          indexed: true
          messages: 15443
          size: 642088366
          start: 1461959937.651054
          topics:
          - {messages: 107, topic: /diagnostics,
             type: diagnostic_msgs/DiagnosticArray}
          - {messages: 8, topic: /oreo/LED_detector_node/switch,
             type: duckietown_msgs/BoolStamped}
          - {messages: 9, topic: /oreo/apriltags_global_node/apriltags,
             type: duckietown_msgs/AprilTags}
          - {messages: 9, topic: /oreo/apriltags_global_node/tags_image,
             type: sensor_msgs/Image}
     ![...]
     

### `dt-logs-thumbnails`: Create thumbnails

Use `dt-logs-thumbnails` to create thumbnails:

    $ dt-logs-thumbnails ![logs query]
    
For example:

    $ dt-logs-thumbnails 20160503164104_ayrton
    Created out-dt-logs-thumbnails/20160503164104_ayrton.thumbnails.jpg
    
Use the option `--write_frames` to write each frame separately.

    $ dt-logs-thumbnails --write_frames 20160503164104_ayrton
    Created out-dt-logs-thumbnails/20160503164104_ayrton/ayrton_camera_node_image_compressed/image-00000.jpg
    Created out-dt-logs-thumbnails/20160503164104_ayrton/ayrton_camera_node_image_compressed/image-00001.jpg
    ![...]
    Created out-dt-logs-thumbnails/20160503164104_ayrton.thumbnails.jpg

Use the option `--all_topics` to do this for all topics, in addition to the camera imge.


### `dt-logs-videos`: Create videos


Use `dt-logs-videos` to create videos:

    $ dt-logs-videos ![logs query]
    
For example:

    $ dt-logs-videos 20160503164104_ayrton
    Created: out-dt-logs-videos/20160503164104_ayrtonvideo.mp4
    

## The selector language {#selector-language}

Here are some examples for the query language ([](#tab:queries)).

Show all the Ferrari logs:

    $ dt-logs-summary vehicle:ferrari

All the logs of length less than 45 s:

<pre><code>&#36; dt-logs-summary "length:&lt;45"</code></pre>

All the invalid logs:

<pre><code>&#36; dt-logs-summary "length:&lt;45,valid:False"</code></pre>

All the invalid logs of length less than 45 s:

<pre><code>&#36; dt-logs-summary "length:&lt;45,valid:False"</code></pre>


<col3 figure-id='tab:queries' class='labels-row1'>
    <figcaption>Query language</figcaption>
    <s>expression</s>
    <s>example</s>
    <s>explanation</s>

    <code>![attribute]:![expr]</code>
    <code>vehicle:ferrari</code>
    <s>Checks that the attribute <code>![[attribute]</code> of the object
        satisfies the expression in <code>![expr]</code></s>

    <code>&gt;![lower bound]</code>
    <code>&gt;10</code>
    <s>Lower bound</s>

    <code>&lt;![upper bound]</code>
    <code>&lt;1</code>
     <s>Upper bound</s>

    <code>![expr1],![expr2]</code>
    <code>&gt;10,&lt;20</code>
    <s>And between two expressions</s>

    <code>![expr1]+![expr2]</code>
    <code>&lt;5+&gt;10</code>
    <s>Or between two expressions</s>

    <code>![pattern]</code>
    <code>*ferrari*</code>
    <s>Other strings are interpreted as wildcard patterns.</s>
</col3>
<style>
#tab\:queries td {
text-align: left;
}
#tab\:queries td:nth-child(3) {

width: 60%;
}
</style>




## Advanced log indexing and generation {#easy_logs-advanced}


### Shuffle  {#easy_logs-shuffle}

`![expr]/shuffle` shuffles the order of the logs in `![expr]`.

Give me all the `oreo` logs, in random order:

    $ dt-logs-summary  vehicle:oreo/shuffle

### Simple indexing  {#easy_logs-indexing}


`![expr]/[i]` takes the i-th  entry.

Give me the first log:

    $ dt-logs-summary  "vehicle:oreo/[0]"

Give me a random log; i.e. the first of a random list.

    $ dt-logs-summary  "vehicle:oreo/shuffle/[0]"

### Complex indexing  {#easy_logs-complex-indexing}


You can use the exact Python syntax for indexing, including `[a:]`, `[:b]`, `[a:b]`, `[a:b:c]`, etc.

Give me three random logs:

    $ dt-logs-summary  "all/shuffle/[:3]"

 
### Time indexing  {#easy_logs-time-indexing}


You can ask for only a part of a log using the syntax:

    ![expr]/{![start]:![stop]}
    ![expr]/{![start]:}
    ![expr]/{:![stop]}

where `![start]` and `![stop]` are in time relative to the start of the log.

For example, "give me all the first 1-second intervals of the logs" is

    all/{:1}

Cut the first 3 seconds of all the logs:

    all/{3:}

Give me the interval between 30 s and 35 s:

    all/{30:35}


## Containerized version of log utils {#dt-logs-containerized}

You can run the commands using th Docker container `duckietown/logs`, by using:

    $ docker run -it duckietown/logs dt-logs-![commands] ![arguments]
    
### Recipe: see the logs

For example, see the logs DB:

    $ docker run -it duckietown/logs dt-logs-summary 

### Recipe: copy cloud logs to the current directory

Copy some logs to the current directory.

    $ docker run -it -v $PWD:/mylogs duckietown/logs dt-logs-copy --outdir /mylogs 20171124170042_yaf 

Note here we mount the current directory as `/mylogs` using the Docker options:

    -v $PWD:/mylogs
    
and then we specify it as the output dir for `dt-logs-copy`.

### Recipe: create thumbnails

Create thumbnails:

    $ docker run -it -v $PWD:/outdir duckietown/logs dt-logs-thumbnails --outdir /outdir 20171124170042_yaf 

### Recipe: create video

Create videos:

    $ docker run -it -v $PWD:/outdir duckietown/logs dt-logs-videos --outdir /outdir 20171124170042_yaf 

### Adding persistence

Mount the directory `/dt-data/DUCKIETOWN_TMP` to have the tmp data persist across invocations:

    $ mkdir -p dt-tmp-data && docker run -it -v $PWD/dt-tmp-data:/dt-data/DUCKIETOWN_TMP duckietown/logs dt-logs-videos 20171124170042_yaf 

