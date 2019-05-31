import json
import time
import math
from enum import Enum

import numpy as np
from numpy.linalg import norm

import rospy

from drawing_control.srv import DrawFileResponse, MoveLaser
from drawing_control.instructions import read_instr, MoveTo, SetPower, Wait
from drawing_control.utils import line_args, intercept, angle, diff_angles, avr_angles


class LaserController1(object):

    _CAL_SCALE = 0.4

    def __init__(self, base_vel, cmd_spacing, resolution, calibration_history_len, position_service):
        self.position_service = position_service
        self.cmd_spacing = cmd_spacing
        self.base_vel = base_vel
        self.resolution = resolution
        self.calibration_history_len = calibration_history_len
        self.last_cmd_time = 0
        self.calibration_init()

    def calibration_init(self):
        self.calibrating = True
        self.last_cmd = None
        self.laser_on = False
        self.set_position_client(0, 0)
        self.laser_on = True
        self.calibration_history = []
        self.instrs = [MoveTo(-self._CAL_SCALE, 0, 1),
                       MoveTo(self._CAL_SCALE, 0, 1),
                       MoveTo(0, -self._CAL_SCALE, 1),
                       MoveTo(0, self._CAL_SCALE, 1)]
                       
    def calibration_check(self):
        pt11, pt12, pt21, pt22 = tuple(np.array([val[1].x, val[1].y]) for val in self.calibration_history)
        m1, c1 = line_args(pt11, pt12)
        m2, c2 = line_args(pt21, pt22)
        # print(pt11, pt12, m1, c1)
        # print(pt21, pt22, m2, c2)
        inter = intercept(m1, c1, m2, c2)
        #print(intercept)
        #print(intercept)
        a1 = angle(pt11, pt12)
        a2 = angle(pt21, pt22)
        diff_angle = diff_angles(a1, a2)
        # print(math.degrees(a1))
        # print(math.degrees(a2))
        # print(math.degrees(diff_angle))
        avr_angle = avr_angles(a1, a2)
        #print(math.degrees(avr_angle))
        scale_x = np.linalg.norm(pt11 - pt12) / (self._CAL_SCALE * 2)
        scale_y = np.linalg.norm(pt21 - pt22) / (self._CAL_SCALE * 2)
        #print(scale_x, scale_y)
        self.calibration = {
            'offset': list(inter),
            'scale': [scale_x, scale_y],
            'rotation': math.degrees(avr_angle)
        }
        print('"calibration": {}'.format(json.dumps(self.calibration)))
        self.calibrating = False

    def set_position_client(self, x, y, laser_on=None):
        if laser_on is None:
            laser_on = self.laser_on
        rospy.wait_for_service(self.position_service)
        try:
            set_position = rospy.ServiceProxy(self.position_service, MoveLaser)
            set_position(x, y, self.laser_on)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s", e)

    def pose_callback(self, pose_msg):
        now = rospy.get_time()
        if now < self.last_cmd_time + self.cmd_spacing:
            return
        # Strategy, initially use derivitive to figure out rought scaling
        # Then Sample the space to make lookup table for interpolation
        # if self.last_pose is not None and self.target_vel != 0:
        #     delta_pose = LaserController1._pose_delta(self.last_pose, pose_msg)
        #     delta_t = pose_msg.stamp - self.last_pose.stamp
        #     self.calibration_history.append({
        #         'target_vel': self.target_vel,
        #         'delta_pose': delta_pose,
        #         'delta_t': delta_t
        #     })
        #     #TODO calculate rotation and scaling based on history
        #     if len(self.calibration_history) > self.calibration_history_len:
        #         self.calibration_history.pop(0)
        if self.calibrating:
            if self.last_cmd is not None:
                self.calibration_history.append((self.last_cmd, pose_msg))
            if len(self.instrs) > 0:
                instr = self.instrs.pop(0)
                self.set_position_client(instr.x, instr.y)
                self.last_cmd_time = now
                self.last_cmd = instr
                return
            self.calibrating = False
            self.calibration_check()
            return
        if len(self.instrs) > 0:
            instr = self.instrs[0]
            if type(instr) == MoveTo:
                delta_pose = np.array([pose_msg.x - instr.x, pose_msg.y - instr.y])
                dist = norm(delta_pose)
                rospy.loginfo('dist: %f', dist)
                if norm(delta_pose) < self.resolution:
                    self.instrs.pop(0)
                    return
                rospy.loginfo('move x: %f y:%f', instr.x, instr.y)
                self.set_position_client(instr.x, instr.y)
                self.last_cmd_time = now
            elif type(instr) == SetPower:
                self.laser_on = instr.is_on
                self.instrs.pop(0)
            elif type(instr) == Wait:
                self.instrs.pop(0)
            

    def draw_file(self, req):
        try:
            with open(req.movement_file) as fd:
                self.instrs = [ read_instr(line) for line in fd.readlines() ]
        except Exception as e:
            rospy.logwarn('draw_file failed: %s', e)
            return DrawFileResponse(False)
        return DrawFileResponse(True)
