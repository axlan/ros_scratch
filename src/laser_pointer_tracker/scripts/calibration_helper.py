#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.server
from laser_pointer_tracker.cfg import Calibration

#! /usr/bin/env python
import sys
import argparse
import cv2


def threshold_image(channel, channels):

    hue_min = rospy.get_param("~hue_min")
    hue_max = rospy.get_param("~hue_max")
    sat_min = rospy.get_param("~sat_min")
    sat_max = rospy.get_param("~sat_max")
    val_min = rospy.get_param("~val_min")
    val_max = rospy.get_param("~val_max")

    rospy.loginfo('hue: %d %d', hue_min, hue_max)

    if channel == "hue":
        minimum = hue_min
        maximum = hue_max
    elif channel == "saturation":
        minimum = sat_min
        maximum = sat_max
    elif channel == "value":
        minimum = val_min
        maximum = val_max

    (t, tmp) = cv2.threshold(
        channels[channel],  # src
        maximum,  # threshold value
        0,  # we dont care because of the selected type
        cv2.THRESH_TOZERO_INV  # t type
    )

    (t, channels[channel]) = cv2.threshold(
        tmp,  # src
        minimum,  # threshold value
        255,  # maxvalue
        cv2.THRESH_BINARY  # type
    )

    # if channel == 'hue':
    #     # only works for filtering red color because the range for the hue
    #     # is split
    #     self.channels['hue'] = cv2.bitwise_not(self.channels['hue'])

def track(self, frame, mask):
    """
    Track the position of the laser pointer.
    Code taken from
    http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
    """
    center = None

    countours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]

    # only proceed if at least one contour was found
    if len(countours) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(countours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        moments = cv2.moments(c)
        if moments["m00"] > 0:
            center = int(moments["m10"] / moments["m00"]), \
                        int(moments["m01"] / moments["m00"])
        else:
            center = int(x), int(y)
        print(radius)
        # only proceed if the radius meets a minimum size
        if radius > 1.8:
            # draw the circle and centroid on the frame,
            cv2.circle(frame, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            # then update the ponter trail
            if self.previous_position:
                cv2.line(self.trail, self.previous_position, center,
                            (255, 255, 255), 2)

    cv2.add(self.trail, frame, frame)
    self.previous_position = center

def detect(frame):
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    channels = {}

    # split the video frame into color channels
    h, s, v = cv2.split(hsv_img)
    channels['hue'] = h
    channels['saturation'] = s
    channels['value'] = v

    # Threshold ranges of HSV components; storing the results in place
    threshold_image("hue", channels)
    threshold_image("saturation", channels)
    threshold_image("value", channels)

    # Perform an AND on HSV components to identify the laser!
    channels['laser'] = cv2.bitwise_and(
        channels['hue'],
        channels['value']
    )
    channels['laser'] = cv2.bitwise_and(
        channels['saturation'],
        channels['laser']
    )

    return channels

bridge = CvBridge()
hue_pub = rospy.Publisher('thresholded_hue', Image, queue_size=10)

def callback(image_message):
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    channels = detect(cv_image)
    #track(frame, channels['laser'])
    hue_out = bridge.cv2_to_imgmsg(channels['hue'], encoding="passthrough")
    hue_pub.publish(hue_out)

def confcallback(config, level):
    return config

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('calibration_helper', anonymous=True)

    srv = Server(TutorialsConfig, confcallback)

    input_image = rospy.get_param("~input_image")
    rospy.Subscriber(input_image, Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
