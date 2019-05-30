
import numpy as np
from numpy.linalg import norm

import rospy

from drawing_control.srv import DrawFileResponse, MoveLaser
from drawing_control.instructions import read_instr, MoveTo, SetPower, Wait

class LaserController1(object):
    def __init__(self, base_vel, cmd_spacing, resolution, calibration_history_len, position_service):
        self.position_service = position_service
        self.cmd_spacing = cmd_spacing
        self.base_vel = base_vel
        self.resolution = resolution
        self.instrs = []
        self.laser_on = False
        self.calibration_history = []
        self.calibration_history_len = calibration_history_len
        self.last_cmd_time = 0

    def set_position_client(self, x, y):
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
