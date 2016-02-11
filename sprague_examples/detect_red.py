#!/usr/bin/env python
"""  Brief demo of image processing using OpenCV Python bindings. 

Author: Nathan Sprague
Version: 3/24/2015
"""
import cv2
import numpy as np


def find_reddest_pixel(img):
    """ Return the pixel location of the reddest pixel in the image.

       Redness is defined as: redness = (r - g) + (r - b)

       Arguments:
            img - height x width x 3 numpy array of uint8 values.

       Returns:
            A tuple (x,y) containg the position of the reddest pixel.
    """
    # HINTS/ADVICE-------------
    # Use a nested for loop here.
    #
    # BE CAREFUL DOING ARITHMETIC WITH UNSIGNED INTEGERS:
    # >>> a = np.array([2], dtype='uint8')
    # >>> b = np.array([3], dtype='uint8')
    # >>> a - b
    #     array([255], dtype=uint8)
    #
    # Reminder:
    # numpy arrays have a "shape" attribute that stores the layout:
    #    img.shape[0] - rows
    #    img.shape[1] - columns
    #    img.shape[2] - color channels

    max_red = -1
    max_red_pos = None

    for x in range(img.shape[1]):
        for y in range(img.shape[0]):

            r = int(img[y, x, 2])
            g = int(img[y, x, 1])
            b = int(img[y, x, 0])

            redness = (r - g) + (r - b)

            if redness > max_red:
                max_red = redness
                max_red_pos = (x, y)

    return max_red_pos


def find_reddest_pixel_fast(img):
    """ Return the pixel location of the reddest pixel in the image.

       Redness is defined as: redness = (r - g) + (r - b)

       Arguments:
            img - height x width x 3 numpy array of uint8 values.

       Returns:
            A tuple (x,y) containg the position of the reddest pixel.
    """
    img = np.array(img, dtype=int)
    r = img[:, :, 2]
    g = img[:, :, 1]
    b = img[:, :, 0]
    redness = (r - g) + (r - b)
    return cv2.minMaxLoc(redness)[3]

def camera_loop():
    """
    Find and mark the reddest pixel in the video stream.
    """
    width = 320
    height = 240

    # Tell OpenCV which camera to use:
    capture = cv2.VideoCapture(0)

    # Set up image capture to grab the correct size:
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)


    while True:
        # Grab the image from the camera.
        success, img = capture.read()

        # Find the most-red pixel:
        red_pixel = find_reddest_pixel_fast(img)

        # Draw a circle on the red pixel.
        # http://docs.opencv.org/modules/core/doc/drawing_functions.html
        cv2.circle(img, red_pixel, 5, (0, 255, 0), -1)

        cv2.imshow("Image", img)
        c = cv2.waitKey(33)

if __name__ == "__main__":
    camera_loop()
