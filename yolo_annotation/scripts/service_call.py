#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from denso_srvs.srv import tsuchida_sama, tsuchida_samaRequest, tsuchida_samaResponse

rospy.init_node("call_cline")
rospy.wait_for_service("tsuchida_service")
try:
    get_mes = rospy.ServiceProxy("tsuchida_service", tsuchida_sama)
    resp1 = get_mes(True)
    tsu = tsuchida_samaResponse()
    print(resp1.messa.data)
except rospy.ServiceException as e:
    print(e)


