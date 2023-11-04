#!/usr/bin/env python3

from cv2 import add
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

import numpy as np
import cv2
import cv_bridge
import os
import yaml

# Global vars. initial values
height = 0
width = 0


# parameter for white lane
hue_white_l = 0
hue_white_h = 179
saturation_white_l = 0
saturation_white_h = 35
lightness_white_l = 215
lightness_white_h = 255

# parameter for yellow lane
hue_yellow_l = 10
hue_yellow_h = 127
saturation_yellow_l = 70
saturation_yellow_h = 255
lightness_yellow_l = 95
lightness_yellow_h = 255

lower_white = np.array([hue_white_l, saturation_white_l, lightness_white_l])
upper_white = np.array([hue_white_h, saturation_white_h, lightness_white_h])

lower_yellow = np.array([hue_yellow_l, saturation_yellow_l, lightness_yellow_l])
upper_yellow = np.array([hue_yellow_h, saturation_yellow_h, lightness_yellow_h])

reliability_white_line = 100
reliability_yellow_line = 100

distortion_coef = np.array([0.0, 0.0, 0.0, 0.0, 0.000000])
camera_matrix = np.array([[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000]])
newcameramatrix = np.array([[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000]])

      

# Create a bridge between ROS and OpenCV
bridge = cv_bridge.CvBridge()
       

def get_white_lane(image):
    """
    Tune the saturation and brightness of the image
    """
    global reliability_white_line
    global hue_white_l
    global hue_white_h
    global saturation_white_l
    global saturation_white_h
    global lightness_white_l
    global lightness_white_h
    global lower_white 
    global upper_white 
    
    global height

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    res = cv2.bitwise_and(image, image, mask = mask)

    res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    
    #Remove white dots from image 

    # convert to binary by thresholding
    ret, binary_map = cv2.threshold(res,127,255,0)

    # do connected components processing
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map)

    #get CC_STAT_AREA component as stats[label, COLUMN] 
    areas = stats[1:,cv2.CC_STAT_AREA]

    result = np.zeros((labels.shape), np.uint8)

    for i in range(0, nlabels - 1):
        if areas[i] >= 100:   #keep
            result[labels == i + 1] = 255

    fraction_num = np.count_nonzero(mask)
    if fraction_num > 35000:
        if lightness_white_l < 250:
                lightness_white_l += 5
    elif fraction_num < 5000:
        if lightness_white_l > 50:
            lightness_white_l -= 5

    how_much_short = 0
    for i in range(0, height):
        if np.count_nonzero(mask[i,::]) > 0:
            how_much_short += 1
    how_much_short = height - how_much_short

    if how_much_short > 100:
        if reliability_white_line >= 5:
            reliability_white_line -= 5
    elif how_much_short <= 100:
        if reliability_white_line <= 99:
            reliability_white_line += 5
            
    result = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)  

    return result

def get_yellow_lane(image):
    global reliability_yellow_line
    global hue_yellow_l
    global hue_yellow_h
    global saturation_yellow_l
    global saturation_yellow_h
    global lightness_yellow_l
    global lightness_yellow_h
    global lower_yellow 
    global upper_yellow 
    

    global crop_w_start
    global height
    

    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    res = cv2.bitwise_and(image, image, mask = mask)

    res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    
    #Remove white dots from image 

    # convert to binary by thresholding
    ret, binary_map = cv2.threshold(res,127,255,0)

    # do connected components processing
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map)

    #get CC_STAT_AREA component as stats[label, COLUMN] 
    areas = stats[1:,cv2.CC_STAT_AREA]

    result = np.zeros((labels.shape), np.uint8)

    for i in range(0, nlabels - 1):
        if areas[i] >= 100:   #keep
            result[labels == i + 1] = 255

    fraction_num = np.count_nonzero(mask)
    if fraction_num > 35000:
        if lightness_yellow_l < 250:
                lightness_yellow_l += 5
    elif fraction_num < 5000:
        if lightness_yellow_l > 50:
            lightness_yellow_l -= 5

    how_much_short = 0
    for i in range(0, height):
        if np.count_nonzero(mask[i,::]) > 0:
            how_much_short += 1
    how_much_short = height - how_much_short

    if how_much_short > 100:
        if reliability_yellow_line >= 5:
            reliability_yellow_line -= 5
    elif how_much_short <= 100:
        if reliability_yellow_line <= 99:
            reliability_yellow_line += 5
          
          
    result = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)  
    height, width, ok = result.shape
    for i in range(0, height):
        for j in range(0, width):
            if result[i,j].sum() != 0:
                result[i,j] = [0, 192, 255]

          
    return result

def image_callback(msg):
    """
    Function to be called whenever a new Image message arrives.
    """
    global node
    global publisher
    global camera_matrix
    global newcameramatrix
    
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
    
    # Wait for the first image to be received
    if type(image_input) != np.ndarray:
        node.get_logger().error('No input image')
        return
    
    # copy image to still have the original image and modify the copy
    image = image_input.copy()

    image = cv2.undistort(image, camera_matrix, distortion_coef, None, newcameramatrix)

    filtered_image_white = get_white_lane(image)
    filtered_image_yellow = get_yellow_lane(image)
    
    
    
    detected_image = cv2.addWeighted(filtered_image_yellow, 1, filtered_image_white, 1, 0.0)
    


   
   

    
    # publish image with the detected lines
    # node.get_logger().info('Publishing image with detected lines')
    publisher.publish(bridge.cv2_to_imgmsg(np.array(detected_image)))


def main():

    rclpy.init()

    global node
    global publisher
    
    global camera_matrix
    global newcameramatrix
    global height

    node = Node('detect_lanes')
    node.get_logger().info("start detect_lanes")

    try:
        camera_intrinsics_file = cv2.FileStorage("install/autonomros_lane_detection/share/autonomros_lane_detection/params/intrinsics.yml", cv2.FILE_STORAGE_READ)
    except:
        print("camera intrinsics not loaded!")

    camera_matrix = camera_intrinsics_file.getNode("camera_matrix").mat()
    distortion_coef = camera_intrinsics_file.getNode("distortion_coefficients").mat()
    width = int(camera_intrinsics_file.getNode("image_width").real())
    height = int(camera_intrinsics_file.getNode("image_height").real())
    print("camera_matrix: \n" , camera_matrix)
    print("distorstion_coefficients: \n" , distortion_coef)
    camera_intrinsics_file.release()
    # calculate undistorted camera matrix
    newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coef, (width, height), 1, (width, height))
    print("undistorted camera matrix: \n" , newcameramatrix)

    #subscribe to camera for getting image of the road
    subscription = node.create_subscription(Image, '/camera_sensor1/image_raw', image_callback, rclpy.qos.qos_profile_sensor_data)
    #publish black image with detected lanes
    publisher = node.create_publisher(Image, 'detected_lanes/image', rclpy.qos.qos_profile_system_default)

    rclpy.spin(node)


try:
    main()
except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
    pass