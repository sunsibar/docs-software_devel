# Exercise: Instagram filters  {#exercise-instagram status=beta}

Assigned: Andrea Daniele

## Skills learned

* Image pixel representation;
* Image manipulation;
* The idea that we can manipulate operations as objects, and refer to them (higher-order computation);
* The idea that we can compose operations, and sometimes the operations do commute,
  while sometimes they do not.

## Instructions

Create `dt-instagram` as specified below.

## Specification for `dt-instagram`

Write a program `dt-instagram` that applies a list of filters to an image.

The syntax to invoke the program is:

    $ dt-instagram ![image in] ![filters] ![image out]

where:

- `![image in]` is the input image;
- `![filters]` is a string, which is a colon-separated list of filters;
- `![image out]` is the output image.

The list of filters is given in [](#instagram-filters).

For example, the result of the command:

    $ dt-instagram image.jpg flip-horizontal:grayscale out.jpg

is that `out.jpg` contains the input image, flipped and than converted to grayscale.

Because these two commute, this command gives the same output:

    $ dt-instagram image.jpg grayscale:flip-horizontal out.jpg



### List of filters {#instagram-filters}

Here is the list of possible values for the filters, and their effect:

- `flip-vertical`: flips the image vertically
- `flip-horizontal`: flips the image horizontally
- `grayscale`: Makes the image grayscale
- `sepia`: make the image sepia


## Useful new APIs

### User defined filters

In OpenCV it is possible to define custom filters and apply them to an image.
A linear filter (e.g., sepia) is defined by a linear 9-dimensional kernel.
The `sepia` filter is defined as:

\[
K_{sepia} =
\begin{bmatrix}
    0.272 & 0.534 & 0.131 \newline
    0.349 & 0.686 & 0.168 \newline
    0.393 & 0.769 & 0.189
\end{bmatrix}
\]

A linear kernel describing a color filter defines a linear transformation in the
color space. A transformation can be applied to an image in OpenCV by using the function
[`transform()`](http://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#cv2.transform).
