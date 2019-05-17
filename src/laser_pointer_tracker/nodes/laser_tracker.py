#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.server
from laser_pointer_tracker.cfg import ThresholdsConfig # pylint: disable=import-error

#! /usr/bin/env python
import sys
import argparse
import cv2
import numpy as np

class LaserTracker(object):

    _COMPONENT_NAMES = ['hue', 'sat', 'val']

    def __init__(self):

        self.bridge = CvBridge()
        topic_base = '{}/'.format(rospy.get_name())
        self.components = {}
        for component_name in LaserTracker._COMPONENT_NAMES:
            component = {}
            component['pub'] = rospy.Publisher(topic_base + 'thresholded_' + component_name, Image, queue_size=10)
            component['min'] = None
            component['max'] = None
            self.components[component_name] = component
        self.hue_pub = rospy.Publisher(topic_base + 'thresholded_combined', Image, queue_size=10)
        self.tracking_image_pub = rospy.Publisher(topic_base + 'tracking_image', Image, queue_size=10)
        self.radius_min = None
        self.radius_max = None
        self.enable_cal_out = False
        dynamic_reconfigure.server.Server(ThresholdsConfig, self._threshold_conf_callback)
        self.previous_position = None
        self.trail = None

    def _threshold_conf_callback(self, config, level):
        for component_name in LaserTracker._COMPONENT_NAMES:
            self.components[component_name]['min'] = config[component_name + '_min']
            self.components[component_name]['max'] = config[component_name + '_max']
        # Scale hue from 0-358 to 0-179
        self.components['hue']['min'] /= 2
        self.components['hue']['max'] /= 2
        self.radius_min = config['radius_min']
        self.radius_max = config['radius_max']
        self.enable_cal_out = config.get('enable_cal_out', False)
        return config

    def _threshold_image(self, channel, channels):
        minimum = self.components[channel]['min']
        maximum = self.components[channel]['max']
        wrapped = channel == "hue" and maximum < minimum
        if wrapped:
            minimum = self.components[channel]['max']
            maximum = self.components[channel]['min']
        if minimum == 0:
            channels[channel][np.where(channels[channel] == 0)] = 1
        (_, tmp) = cv2.threshold(
            channels[channel],  # src
            maximum,  # threshold value
            0,  # we dont care because of the selected type
            cv2.THRESH_TOZERO_INV  # t type
        )
        (_, channels[channel]) = cv2.threshold(
            tmp,  # src
            minimum,  # threshold value
            255,  # maxvalue
            cv2.THRESH_BINARY  # type
        )
        if wrapped:
            # only works for filtering red color because the range for the hue
            # is split
            channels[channel] = cv2.bitwise_not(channels[channel])

    def _track(self, frame, mask):
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
            # only proceed if the radius meets a minimum size
            if radius > self.radius_min and radius < self.radius_max:
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

    def _detect(self, frame):
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        channels = {}

        # split the video frame into color channels
        h, s, v = cv2.split(hsv_img)
        values = [h, s, v]

        for comp, component_name in zip(values, LaserTracker._COMPONENT_NAMES):
            channels[component_name] = comp
            # Threshold ranges of HSV components; storing the results in place
            self._threshold_image(component_name, channels)

        # Perform an AND on HSV components to identify the laser!
        channels['laser'] = cv2.bitwise_and(
            channels[LaserTracker._COMPONENT_NAMES[0]],
            channels[LaserTracker._COMPONENT_NAMES[1]]
        )
        channels['laser'] = cv2.bitwise_and(
            channels[LaserTracker._COMPONENT_NAMES[2]],
            channels['laser']
        )
        return channels


    def callback(self, image_message):
        if self.trail is None:
            self.trail = np.zeros((image_message.height, image_message.width, 3),
                                  np.uint8)
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
        channels = self._detect(cv_image)
        self._track(cv_image, channels['laser'])
        tracking_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.tracking_image_pub.publish(tracking_image)
        if self.enable_cal_out:
            for component_name in LaserTracker._COMPONENT_NAMES:
                out_img = self.bridge.cv2_to_imgmsg(channels[component_name], encoding="passthrough")
                self.components[component_name]['pub'].publish(out_img)
            # Merge the HSV components back together.
            hsv_image = cv2.merge([ channels[i] for i in LaserTracker._COMPONENT_NAMES ])
            out_img = self.bridge.cv2_to_imgmsg(hsv_image, encoding="bgr8")
            self.hue_pub.publish(out_img)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('calibration_helper', anonymous=True)

    laser_tracker = LaserTracker()

    input_image = rospy.get_param("~input_image")
    rospy.Subscriber(input_image, Image, laser_tracker.callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
