---
layout: single
title: "DAT160 Robot Vision"
permalink: /courses/dat160/robot-vision
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---

On this page you can learn about robot vison, with some simple exsamples on how to implement it in python.

## Digital image
In OpenCV:

```python
# Load single color image:
image = cv2.imread('img/ball01.jpg')

# numpy array:
image.shape
# (667, 500, 3)
# width(px), height(px), layers

cv2.imshow("Original", image)
cv2.waitKey(0)
```

- Digital images are typically stored in RGB format
- In OpenCV the default order is not RGB but BGR !!

Plotting the layers separately:
```python
cv2.imshow("B", image[:,:,0])
cv2.imshow("G", image[:,:,1])
cv2.imshow("R", image[:,:,2])
cv2.waitKey(0)
```

## Convert color space
- In OpenCV we can easily convert the color space of an image:

  ```python
  hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  lab_image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

  image = cv2.cvtColor(image, cv2.COLOR_LAB2BGR)
  ```

- Layers in CIELAB:
```python
cv2.imshow("L", lab_image[:,:,0])
cv2.imshow("A", lab_image[:,:,1])
cv2.imshow("B", lab_image[:,:,2])
cv2.waitKey(0)
```

- Layers in HSV:
```python
cv2.imshow("H", hsv_image[:,:,0])
cv2.imshow("S", hsv_image[:,:,1])
cv2.imshow("V", hsv_image[:,:,2])
cv2.waitKey(0)
```

- The color space we are working in matters, since we perform all our algorithms on the layers
of the digital image.
- Some features of the image may be much more characteristic in one color space than the in
another.
- We will see this clearly when implementing color blob detection

## Image Processing
- Image processing is a computational process that transforms one or more input images into
an output image
- Frequently used to make images more appealing
- Enhancing of imperfect images
- In robotics and computer vision, image processing is often the basis for feature extraction

- Some algorithm categories
    - Pixel-wise on single image: monadic operations
    - Pixel-wise on a pir of images: dyadic operations
    - On local groups of pixels: spatial operations
    - Shape changing operations

## Pixel Value Distribution
- Min, max, mean, median, standard deviation of pixel values
```python
image.mean()
160.36176011994004
image[:,:,0].mean()
152.46574812593704
image[:,:,1].mean()
161.83708245877062
image[:,:,2].mean()
166.78244977511244
image[:,:,1].max()
214
image[:,:,1].min()
27
```


## calcHist
```python
histr = cv2.calcHist([image], [0], None, [256], [0, 256])
plt.plot(histr)
plt.show()
```
Get an impression of the pixel value distribution in an image by plotting it in a histogram showing the frequency of pixel values. For color images: one histogram per layer.
```python
cv2.calcHist(images, channels, mask, histSize, ranges[, hist[, accumulate]])
```
```images```: it is the source image of type uint8 or float32. it should be given in square brackets, ie, "[img]".

```channels```: it is also given in square brackets. It is the index of channel for which we calculate histogram. For example, if input is grayscale image, its value is [0]. For color image, you can pass [0], [1] or [2] to calculate histogram of blue, green or red channel respectively.

```mask```: mask image. To find histogram of full image, it is given as "None". But if you want to find histogram of particular region of image, you have to create a mask image for that and give it as a mask.

```histSize```: this represents our BIN count. Need to be given in square brackets. For full scale, we pass [256].

```ranges```: this is our RANGE. Normally, it is [0,256].

## Monadic Operations
- Very common: Thresholding
```python
threshold_image = cv2.threshold(image[:,:,0], 110, 255, cv2.THRESH_BINARY)
```
All pixels where the intensity is greater 110 in the blue channel become 
True (255) and all less become False (0). The output is a binary image

- To threshold on all channels simultaneously:
```python
threshold_image = cv2.inRange(image, np.array([5, 20, 80]), np.array([60, 130, 190]))
```
All pixels where the intensities of all channels fall between the 
lower and upper bound become True, all other ones false.

## Threshold
```python
cv2.threshold(src, thresh, maxval, type[, dst])
```
```src```: This is the source image, which should be a grayscale image

```thresh```: This is the threshold value witch is used to classify the 
pixel intensities in the grayscale image.

```maxval```: This is the value to be given if pixel value is more than 
(sometimes less than, depending on the type of thresholding) the threshold value.

```type```: This is the type of threshold to be applied. Some types are: 
- ```cv2.THRESH_BINARY```
- ```cv2.THRESH_BINARY_INV```
- ```cv2.THRESH_TRUNC```
- ```cv2.THRESH_TOZERO```
- ```cv2.THRESH_TOZERO_INV```

This function returns two outputs:
- ```retval```: This is the threshold that was used.
- ```dst```: This is the thresholded image.

## inRange
```python
cv2.inRange(src, lowerb, upperb[, dst])
```
- ```src```: This is the source array or image.

- ```lowerb```: This is the inclusive lower boundary array or a scalar.

- ```upperb```: This is the inclusive upper boundary array or a scaler.

The function checks if elements in the ```src``` array lie between ```lowerb``` and ```upperb``` 
(inclusive). If they do, the function sets the corresponding element in the output array to 255 
(white). If they dont't, the function sets the corresponding element in the output array to 0 (black).

## Dyadic Operations
Common examples: arithmetic operators such as addition, subtraction, element-wise
multiplication. Also logic operations such as AND, OR, XOR. Here we apply a mask to an image with the bitwise AND operator:

```python
thresholded_image = cv2.inRange(image, np.array([5, 20, 80]), np.array([60, 130, 190]))
res = cv2.bitwise_and(image, image, mask=thrasholded_image)
cv2.imshow("Result", res)
```
## Blob detection
OpenCV comes with a blob detector that is easy to use:
- We first define the filter parameters

  ```python
  params = cv2.SimpleBlobDetector_Params()

  # Filter by Area.
  params.filterByArea = True
  params.minArea = 150
  params.maxArea = 6000

  # Filter by Circularity
  params.filterByCircularity = False
  params.minCircularity = 0.1

  # Filter by Convexity
  params.filterByConvexity = False
  params.minConvexity = 0.87

  # Filter by Inertia
  params.filterByInertia = False
  params.minInertiaRatio = 0.01
  ```

- With the parameters object we create the blob detector

  ```python
  # Create a detector with the parameters
  detector = cv2.SimpleBlobDetector_create(params)
  ```

- Detect blobs and visualize them

  ```python
  # Create a detector with the parameters
  detector = cv2.SimpleBlobDetector_create(params)

  # Detect blobs
  keypoints = detector.detect(~thresholded_image)

  im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

  cv2.imshow("Blobs", im_with_keypoints)
  cv2.waitKey(0)
  ```

- Keypoints:
  The blob detector returns a list of keypoints. Each keypoint is a special structure whitch contains information about the detected feature. Here's a brief overview of the properties of a keypoint:

  - ```pt```: The coordinates of the detected feature in the format (x, y).
  - ```size```: The diameter of the meaningful keypoint neighborhood.
  - ```angle```: Computed orientation of the keypoint (-1 if not applicable); it's the angle that the keypoint vector is pointing in.
  - ```response```: The response by witch the most strong keypoints have been selected. Can be used for futher sorting or subsampling.
  - ```ovtave```: The octave (pyramid layer) from witch the keypoint has been extracted.
  - ```class_id```: Can be used to cluster keypoints by an  object the belong to.
