#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

def talker():
    rospy.init_node('joint_trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)
    #pub = rospy.Publisher('/joint_group_position_controller/command', Float64MultiArray, queue_size=10)
    rospy.sleep(1.5)

    #tes = Float64MultiArray()
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()


    msg.joint_names = [ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" ]
    msg.points = [JointTrajectoryPoint() for i in range(1)]
    msg.points[0].positions = [0.220046, -1.6059, 1.24065, -1.14717, -1.5684, -1.57072];
    msg.points[0].time_from_start = rospy.Time(0.5)
    #tes.data = [math.pi/2, -math.pi/6.0, -math.pi/2, -math.pi/3, -math.pi/2, 0.0]
    #tes.data = [-math.pi/2, -2.8 * math.pi/6, -math.pi/3, -1*math.pi/1.5, math.pi/2, -math.pi/2];

    pub.publish(msg)
    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass