#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node("tsuf")
pub = rospy.Publisher("tsuchida_pub", String, queue_size=10)
count = 0
rate = rospy.Rate(100)
while not rospy.is_shutdown():
    data = String()
    data.data = "tsuchida_" + str(count)
    count = count + 1
    pub.publish(data)
    rate.sleep()