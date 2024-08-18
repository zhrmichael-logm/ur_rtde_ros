import rospy
import numpy as np

from ur_rtde_ros.srv import set_mode, moveL, schedule_waypoint, get_states

rospy.init_node("ros_example")

rospy.wait_for_service("/set_mode")
rospy.wait_for_service("/moveL")
rospy.wait_for_service("/schedule_waypoint")
rospy.wait_for_service("/get_tcp_states")
set_mode = rospy.ServiceProxy("/set_mode", set_mode)
moveL = rospy.ServiceProxy("/moveL", moveL)
schedule_waypoint = rospy.ServiceProxy("/schedule_waypoint", schedule_waypoint)
get_tcp_states = rospy.ServiceProxy("/get_tcp_states", get_states)


# Set the robot to MOVEL mode
set_mode("MOVEL")

curr_tcp_pose = np.array(get_tcp_states().pose)
print(f"Current TCP pose: {curr_tcp_pose}")

home_tcp_euler = [3.14159265, 0.0, 0.0]
home_tcp_position = [-0.475, 0.210, 0.400]
home_pose = home_tcp_position + home_tcp_euler
vel = 0.05
acc = 0.05

moveL(home_pose, vel, acc)

curr_tcp_pose = np.array(get_tcp_states().pose)
print(f"Current TCP pose: {curr_tcp_pose}")

set_mode("SERVOL")
rospy.sleep(0.5)
freq = 30
rate = rospy.Rate(freq)
dt = 1.0 / freq
ts = rospy.get_time()
for i in range(60):
    _ = np.array(get_tcp_states().pose)
    curr_tcp_pose[0] -= 0.001
    curr_tcp_pose[3] -= 0.3/180*3.14159265
    command_time = rospy.get_time() + dt
    schedule_waypoint(list(curr_tcp_pose), command_time)
    rate.sleep()
te = rospy.get_time()
print(f"actual avg frequency: {60/(te-ts)}")

rospy.sleep(2)
curr_tcp_pose = np.array(get_tcp_states().pose)
print(f"Current TCP pose: {curr_tcp_pose}")
set_mode("IDLE")
print("Done")
