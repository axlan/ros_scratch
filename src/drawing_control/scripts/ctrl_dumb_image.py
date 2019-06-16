#!/usr/bin/env python

import sys
import time

import cv2
import rospy


from drawing_control.srv import DrawFileResponse, MoveLaser

position_service = '/lasernode1/move_laser'

def set_position_client(cur_x, cur_y, laser_on):
    rospy.loginfo('Moving to %f %f : %d', cur_x, cur_y, laser_on)
    rospy.wait_for_service(position_service)
    try:
        set_position = rospy.ServiceProxy(position_service, MoveLaser)
        set_position(cur_x, cur_y, laser_on)
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s", e)
    time.sleep(0.1)

def draw_image(image_file, draw_down=True):
    im = cv2.imread(image_file, cv2.COLOR_BGR2GRAY)
    h, w = im.shape[:2]
    step = 1.0 / float(w)
    for r in range(0, w):
        for c in range(0, w):
            if im[r,c][0] == 0:
                sys.stdout.write('0 ')
            else:
                sys.stdout.write('1 ')
        sys.stdout.write('\n')
    for c in range(0, w):
        x = 0.5 - step * float(c)
        set_position_client(x, -0.5, False)
        r = 0
        while r < h:
            skip = False
            while 255 == im[r,c][0]:
                r += 1
                if r == h:
                    skip = True
                    break
            if skip:
                continue
            y = -0.5 + step * float(r - 1)
            set_position_client(x, y, False)
            while r < h and 0 == im[r,c][0]:
                r += 1
            end = r - 1
            y = -0.5 + step * float(r - 1)
            set_position_client(x, y, True)
    set_position_client(0, 0, False)



def main():
    rospy.init_node('ctrl_dumb_image', anonymous=True)
    image_file = sys.argv[1]
    draw_image(image_file)

if __name__ == "__main__":
    main()


