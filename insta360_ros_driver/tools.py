#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import CompressedImage, Image
import numpy as np

def rotate_image(image, angle):
    height, width = image.shape[:2]
    rotation_matrix = cv2.getRotationMatrix2D((width/2, height/2), angle, 1)
    rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))
    return rotated_image
def split_image(image):
    height, width = image.shape[:2]
    mid_point = width // 2
    front_image = image[:, :mid_point]
    back_image = image[:, mid_point:]

    return back_image, front_image

def undistort_image(image, K, D):
    h, w = image.shape[:2]
    new_K = K.copy()
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_32FC1)
    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)
    return undistorted_img

def compress_image_to_msg(image, timestamp):
    _, buffer = cv2.imencode('.jpg', image)
    image_msg = CompressedImage()
    image_msg.header.stamp = timestamp
    image_msg.format = 'jpeg'
    image_msg.data = buffer.tobytes()
    return image_msg

def image_to_msg(image, timestamp):
    image_msg = Image()
    image_msg.header.stamp = timestamp
    image_msg.height = image.shape[0]
    image_msg.width = image.shape[1]
    image_msg.encoding = 'bgr8'
    image_msg.is_bigendian = 0
    image_msg.step = 3 * image.shape[1]
    image_msg.data = image.tobytes()
    return image_msg