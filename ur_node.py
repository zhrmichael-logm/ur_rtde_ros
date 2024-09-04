#!/usr/bin/env python3
import numpy as np
import time
import rospy

# import scipy.interpolate as si
# import scipy.spatial.transform as st

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

from ur_rtde_ros.srv import set_mode, moveL, schedule_waypoint, get_states
from utils.pose_trajectory_interpolator import PoseTrajectoryInterpolator


MODE_MAP = {
    "IDLE": 0,
    "MOVEL": 1,
    "SERVOL": 2,
    "STOP": 3,
}


class URNode:
    def __init__(self):
        self.ip = rospy.get_param("/ur_rtde/ip", "10.42.0.123")
        self.frequency = float(rospy.get_param("/ur_rtde/frequency", 125.0))

        # self.home_tcp_euler = np.array(rospy.get_param("/ur_rtde/home_tcp_euler", [180, 0, 0]))
        # self.home_tcp_position = np.array(rospy.get_param("/ur_rtde/home_tcp_position", [-0.475, 0.210, 0.400]))
        self.servo_lookahead_time = float(rospy.get_param("/ur_rtde/servo_lookahead_time", 0.2))
        self.servo_gain = float(rospy.get_param("/ur_rtde/servo_gain", 300))
        self.max_pos_speed = float(rospy.get_param("/ur_rtde/max_pos_speed", 3.0))
        self.max_rot_speed = float(rospy.get_param("/ur_rtde/max_rot_speed", 1.0))

        # Connect to the robot
        self.ur_control = RTDEControlInterface(self.ip, self.frequency)
        self.ur_receive = RTDEReceiveInterface(self.ip, self.frequency)
        if not self.ur_control.isConnected():
            raise Exception("Unable to connect to RTDEControlInterface")
        if not self.ur_receive.isConnected():
            raise Exception("Unable to connect to RTDEReceiveInterface")

        # Services
        rospy.Service("/ur_node/set_mode", set_mode, self.set_mode)
        rospy.Service("/ur_node/moveL", moveL, self.move_to_pose)
        rospy.Service("/ur_node/schedule_waypoint", schedule_waypoint, self.schedule_waypoint)
        rospy.Service("/ur_node/get_tcp_states", get_states, self.get_tcp_states)
        rospy.Service("/ur_node/get_joint_states", get_states, self.get_joint_states)

        self.mode = MODE_MAP["IDLE"]
        self.last_waypoint_time = None

    def close(self):
        self.ur_control.servoStop()
        self.ur_control.stopScript()
        self.ur_control.disconnect()
        self.ur_receive.disconnect()

    def set_mode(self, req):
        assert isinstance(req.mode, str), "Mode must be a string"
        if req.mode not in MODE_MAP.keys():
            return {"success": False}
        self.mode = MODE_MAP[req.mode]
        return {"success": True}

    def get_tcp_states(self, req):
        tcp_pose = self.ur_receive.getActualTCPPose()
        tcp_speed = self.ur_receive.getActualTCPSpeed()
        return {"pose": tcp_pose, "speed": tcp_speed}

    def get_joint_states(self, req):
        joint_pos = self.ur_receive.getActualQ()
        joint_speed = self.ur_receive.getActualQd()
        return {"pose": joint_pos, "speed": joint_speed}

    def move_to_pose(self, req):
        assert len(req.pose) == 6
        if self.mode != MODE_MAP["MOVEL"]:
            return {"success": False}
        target_pose = np.array(req.pose)
        target_vel = float(req.vel)
        target_acc = float(req.acc)
        assert self.ur_control.moveL(target_pose, target_vel, target_acc, False)
        return {"success": True}

    def schedule_waypoint(self, req):
        if self.mode != MODE_MAP["SERVOL"]:
            return {"success": False}
        target_pose = np.array(req.pose)
        target_time = float(req.target_time)
        # translate global time to monotonic time
        # target_time = time.monotonic() - time.time() + target_time
        # curr_time = t_now + self.dt
        curr_time = rospy.get_time()
        self.pose_interp = self.pose_interp.schedule_waypoint(
            pose=target_pose,
            time=target_time,
            max_pos_speed=self.max_pos_speed,
            max_rot_speed=self.max_rot_speed,
            curr_time=curr_time,
            last_waypoint_time=self.last_waypoint_time,
        )
        self.last_waypoint_time = target_time
        return {"success": True}

    def servo_loop(self):
        dt = 1.0 / self.frequency
        while not rospy.is_shutdown():

            if self.mode != MODE_MAP["SERVOL"]:
                # idle servo loop
                rospy.sleep(dt)
            else:
                curr_time = rospy.get_time()
                curr_pose = self.ur_receive.getActualTCPPose()
                self.last_waypoint_time = curr_time
                self.pose_interp = PoseTrajectoryInterpolator(times=[curr_time], poses=[curr_pose])
                while not rospy.is_shutdown() and self.mode == MODE_MAP["SERVOL"]:
                    # ts = time.time()
                    t_start = self.ur_control.initPeriod()
                    t_now = rospy.get_time()
                    pose_command = self.pose_interp(t_now)
                    vel, acc = 0.5, 0.5
                    assert self.ur_control.servoL(
                        pose_command, vel, acc, dt, self.servo_lookahead_time, self.servo_gain
                    )
                    self.ur_control.waitPeriod(t_start)
                    # rospy.loginfo(f"loop actual dt: {(time.time() - ts)*1000:.3f}ms")
                self.ur_control.servoStop(a=0.3)


def main():
    rospy.init_node("ur_node", anonymous=True)
    node = URNode()
    try:
        node.servo_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.close()


if __name__ == "__main__":
    main()
