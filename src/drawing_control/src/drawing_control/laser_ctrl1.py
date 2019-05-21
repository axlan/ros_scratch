

import rospy
from geometry_msgs.msg import Twist, Vector3

from drawing_control.srv import DrawFileResponse
from drawing_control.instructions import read_instr

class LaserController1(object):
    def __init__(self, base_vel, resolution, calibration_history_len, vel_out_topic):
        self.heading = 0.0
        self.scaling = Twist(linear=Vector3(1, 1, 1),
                             angular=Vector3(1, 1, 1))
        self.vel_out_topic = vel_out_topic
        self.resolution = resolution
        self.base_vel = base_vel
        self.instrs = []
        self.target_vel = 0
        self.calibration_history = []
        self.calibration_history_len = calibration_history_len
        self.last_pose = None

    @staticmethod
    def _pose_delta(prev, new):
        return Vector3(new.x - prev.x,
                       new.y - prev.y,
                       0)

    def pose_callback(self, pose_msg):
        if self.last_pose is not None and self.target_vel != 0:
            delta_pose = LaserController1._pose_delta(self.last_pose, pose_msg)
            delta_t = pose_msg.stamp - self.last_pose.stamp
            self.calibration_history.append({
                'target_vel': self.target_vel,
                'delta_pose': delta_pose,
                'delta_t': delta_t
            })
            #TODO calculate rotation and scaling based on history
            if len(self.calibration_history) > self.calibration_history_len:
                self.calibration_history.pop(0)
        if len(self.instrs) > 0:
            #TODO decide on target volicity, and apply calibration
            pass
        self.last_pose = pose_msg

    def draw_file(self, req):
        try:
            with open(req.movement_file) as fd:
                self.instrs = [ read_instr(line) for line in fd.readlines() ]
        except Exception as e:
            rospy.logwarn('draw_file failed: %s', e)
            return DrawFileResponse(False)
        return DrawFileResponse(True)
