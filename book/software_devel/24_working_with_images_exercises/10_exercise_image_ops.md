# Exercise: Basic image operations {#exercise-basic-image status=beta}

Assigned: Andrea Daniele

## Skills learned

- Accessing command line arguments.
- Reading and writing files.
- Working with pixel-based image representations.
- OpenCV and `duckietown_utils` APIs for reading/writing images.


## Instructions

Create an implementation of the program `dt-image-flip0`, specified below.

If this exercise is too easy for you, skip to [](+exercises#exercise-specifications).


## Specification of `dt-image-flip0`

The program `dt-image-flip0` takes as an argument a JPG file with extension `.jpg`:

    $ dt-image-flip0 ![file].jpg

and creates a file called `![file].flipped.jpg` that is flipped around the horizontal axis.


<div figure-id="fig:example1" figure-class="flow-subfigures">
    <img figure-id="subfig:original1" src='image-ops-original.jpg'/>
    <img figure-id="subfig:flip1"     src='image-ops-flip.jpg'/>
</div>

<figcaption id="fig:example:caption">
Example input-output for the program <code>dt-image-flip</code>.
</figcaption>

<figcaption id="subfig:original1:caption">
The original picture.
</figcaption>

<figcaption id="subfig:flip1:caption">
The flipped output
</figcaption>


## Useful APIs

### Load image from file

The OpenCV library provides a utility function called [`imread`](http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#imread)
that loads an image from a file.


### Flip an image

<!-- The OpenCV library provides a utility function called [`flip`](http://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html?highlight=flip#flip)
that flips an image around vertical, horizontal, or both axes. -->

Comment: This is the kind of thing that they need to figure out how to do
with pixels. -AC

### Write an image to a file

The [`duckietown_utils`](+software_devel#duckietown-utils-library)
package provides the utility function [`write_image_as_jpg()`](+software_devel#duckietown_utils-write_image_as_jpg)
that writes an image to a JPEG file.
