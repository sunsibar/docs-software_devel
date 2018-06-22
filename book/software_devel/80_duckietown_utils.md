# Duckietown utility library {#duckietown-utils-library status=draft}

<!-- The functions called `d8n_???` have stable interface and can be reused. -->

## Images

This sections contains the documentation about the utility functions used for
image processing available in the `duckietown_utils` Python package.


### Function `write_image_as_jpg` {#duckietown_utils-write_image_as_jpg}

**Description**: Takes an BGR image and writes it as a JPEG file.

Comment: Are we sure that the encoding is right? -AC


**Prototype:**
```python
write_image_as_jpg( image, filename )
```

**Defined in:**
[`image_writing.py`](https://github.com/duckietown/Software/blob/master/catkin_ws/src/00-infrastructure/duckietown/include/duckietown_utils/image_writing.py).


**Arguments:**
<col3 class="labels-row1">
    <span>Name</span>
    <span>Type</span>
    <span>Description</span>
    <span>image</span>
    <span><a href="https://docs.scipy.org/doc/numpy/reference/generated/numpy.ndarray.html">numpy.ndarray</a></span>
    <span>The BGR image to save as JPEG file.</span>
    <span>filename</span>
    <span><a href="https://docs.python.org/2/library/functions.html#str">str</a></span>
    <span>The path of the JPEG file.</span>
</col3>

**Returns**:
[`None`](https://docs.python.org/2/library/constants.html#None).




### Function `rgb_from_ros` {#duckietown_utils-rgb_from_ros}

**Description**: Takes a ROS message containing an image and returns its RGB representation.

**Prototype:**
```python
rgb_from_ros( msg )
```

**Defined in:**
[`image_conversions.py`](https://github.com/duckietown/Software/blob/master/catkin_ws/src/00-infrastructure/duckietown/include/duckietown_utils/image_conversions.py).


**Arguments:**
<col3 class="labels-row1">
    <span>Name</span>
    <span>Type</span>
    <span>Description</span>
    <span>msg</span>
    <span>
      <a href="http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Image.html">sensor_msgs.Image</a>
      <br/>
      or
      <br/>
      <a href="http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CompressedImage.html">sensor_msgs.CompressedImage</a>
    </span>
    <span>Message containing the image to extract.</span>
</col3>

**Returns**:
[`numpy.ndarray`](https://docs.scipy.org/doc/numpy/reference/generated/numpy.ndarray.html)
:: RGB representation of the image contained in the ROS message `msg`.



### Function `d8_compressed_image_from_cv_image` {#duckietown_utils-d8_compressed_image_from_cv_image}

**Description**: Takes a OpenCV image (BGR format), compresses it and wraps it into a ROS message of type
[`sensor_msgs.CompressedImage`](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CompressedImage.html).

**Prototype:**
```python
d8_compressed_image_from_cv_image( image_cv )
```

**Defined in:**
[`image_jpg_create.py`](https://github.com/duckietown/Software/blob/master/catkin_ws/src/00-infrastructure/duckietown/include/duckietown_utils/image_jpg_create.py).


**Arguments:**
<col3 class="labels-row1">
    <span>Name</span>
    <span>Type</span>
    <span>Description</span>
    <span>image_cv</span>
    <span><a href="https://docs.scipy.org/doc/numpy/reference/generated/numpy.ndarray.html">numpy.ndarray</a></span>
    <span>BGR representation of the image to compress.</span>
</col3>

**Returns**:
[`sensor_msgs.CompressedImage`](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CompressedImage.html)
:: A ROS message containing a compressed version of the input `image_cv`.
