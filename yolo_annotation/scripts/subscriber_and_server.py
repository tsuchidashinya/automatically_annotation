#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from denso_srvs.srv import tsuchida_sama, tsuchida_samaRequest, tsuchida_samaResponse

class sub_and_service():
    def __init__(self):
        service = rospy.Service("tsuchida_service", tsuchida_sama, self.service_callback)
        sub = rospy.Subscriber("tsuchida_pub", String, self.callback)

    def callback(self, msg):
        self.service_tyuukai = msg
        rospy.loginfo(msg.data)
    
    def service_callback(self, req):
        res = tsuchida_samaResponse()
        res.messa = self.service_tyuukai
        return res

rospy.init_node("fyafd")
sub = sub_and_service()
rospy.spin()
        