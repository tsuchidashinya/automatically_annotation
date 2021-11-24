#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import *
import random
from rospy.timer import Rate
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
# from color_cloud_bridge.msg import object_kiriwake
from denso_msgs.msg import object_kiriwake
import math
import numpy as np
from denso_srvs.srv import object_occuluder_service, object_occuluder_serviceRequest, object_occuluder_serviceResponse
from gazebo_msgs.srv import SetModelState, SetModelStateRequest


def distance(x1, y1, x2, y2):
    return math.sqrt(float((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)))

def pose_is_ok(x, y, x_kako, y_kako, kyori, minus, plas):
    ok = True
    if x < minus or x > plas or y < minus or y > plas:
        ok = False
    for i in range(len(x_kako)):
        if distance(x, y, x_kako[i], y_kako[i]) <= kyori:
            ok = False
    return ok

def limit_pose_is_ok(x, y, limit_minus, limit_plus):
    if x < limit_minus or x > limit_plus or y < limit_minus or y > limit_plus:
        return False
    else:
        return True


class annotation_environment(object):
    def __init__(self, model_name):
        cont = 0
        rospy.set_param("move_is_ok", True)
        rospy.set_param("record_is_ok", False)
        rospy.sleep(1.2)
        print("start")
        self.kaisuu = 0
        self.model_name = model_name
        # self.z_coordinamte = 0.17
        # self.z_coordinamte = 0.17
        self.z_coordinamte = rospy.get_param("~z_zahyou", 0)
        self.box_z = 0
        self.occulution_object = object_kiriwake()
        self.occuluder_array = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        self.occuludy_array = [16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31]
        self.decide_occuluder()
        self.first_function()
        

    def first_function(self):
        self.gazebo_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.occuluder_pub = rospy.Publisher("tsuchida_object_occuluder", object_kiriwake, queue_size=10)
        # self.box_move()
        self.execute()

    def object_name_make(self, base_name, object_array):
        object_name = []
        for i in range(len(object_array)):
            strning = base_name + "_" + str(object_array[i])
            object_name.append(strning)
        return object_name

    def object_move(self):
        pose_list = []
        a_1 = 0.155
        a_4 = -0.155
        limit_distance = 0.09
        a1 = np.arange(a_4, a_1, 0.099)
        a2 = np.arange(a_4, a_1, 0.099)
        youso = []
        for l1 in a1:
            for l2 in a2:
                youso.append([round(l1, 4), round(l2, 4)])
        random.shuffle(youso)
        for i in range(len(self.occulution_object.occuluder_name)):
            pose_data = ModelState()
            pose_data.model_name =  self.occulution_object.occuluder_name[i]
           
            x = youso[i][0]
            y = youso[i][1]   
            # z = self.z_coordinamte + 0.075
            z = self.box_z + 0.07
            pose_object = geometry_msgs.msg.Pose()
            pose_object.position.x = x
            pose_object.position.y = y
            pose_object.position.z = z
            self.occulution_object.occuluder_pose.append(pose_object)
            roll = 0
            pitch = 0
            yaw = 0
            quat = quaternion_from_euler(roll, pitch, yaw)
            pose_data.pose.position.x = x
            pose_data.pose.position.y = y
            pose_data.pose.position.z = z
            pose_data.pose.orientation.x = quat[0]
            pose_data.pose.orientation.y = quat[1]
            pose_data.pose.orientation.z = quat[2]
            pose_data.pose.orientation.w = quat[3]
            pose_list.append(pose_data)
        x_kako = []
        y_kako = []
        for i in range(len(self.occulution_object.occuludy_name)):
            pose_data = ModelState()
            pose_data.model_name = self.occulution_object.occuludy_name[i]
            hanni = 0.03
            if i == 0:
                x_zure = random.uniform(-hanni, hanni)
                y_zure = random.uniform(-hanni, hanni)
                x = self.occulution_object.occuluder_pose[i].position.x + x_zure
                y = self.occulution_object.occuluder_pose[i].position.y + y_zure
            else:
                count = 0
                while True:
                    # count = count + 1
                    # print(count)
                    x_zure = random.uniform(-hanni, hanni)
                    y_zure = random.uniform(-hanni, hanni)
                    x = self.occulution_object.occuluder_pose[i].position.x + x_zure
                    y = self.occulution_object.occuluder_pose[i].position.y + y_zure
                    if limit_pose_is_ok(x, y, a_4, a_1):
                        break
            # z = self.z_coordinamte + 0.025
            z = self.box_z + 0.02
            x_kako.append(x)
            y_kako.append(y)
            roll = 0
            pitch = 0
            yaw = 0
            quat = quaternion_from_euler(roll, pitch, yaw)
            pose_data.pose.position.x = x
            pose_data.pose.position.y = y
            pose_data.pose.position.z = z
            pose_data.pose.orientation.x = quat[0]
            pose_data.pose.orientation.y = quat[1]
            pose_data.pose.orientation.z = quat[2]
            pose_data.pose.orientation.w = quat[3]
            pose_list.append(pose_data)

        loop = rospy.Rate(100)
        for i in range(len(pose_list)):
            model_request = SetModelStateRequest()
            model_request.model_state = pose_list[len(pose_list) - 1 - i]
            self.gazebo_client(model_request)
            loop.sleep()
        
    
    def home_move(self):
        loop = rospy.Rate(100)
        com = 0
        pose_list = []
        a_1 = -12
        a_4 = -15
        all_object = []
        for st in self.object_name_make(self.model_name, self.occuluder_array):
            all_object.append(st)
        for st in self.object_name_make(self.model_name, self.occuludy_array):
            all_object.append(st)
        all_object.append(self.model_name)
        for i in range(len(all_object)):
            pose_data = ModelState()
            pose_data.model_name = all_object[i]
            x = random.uniform(a_4, a_1)
            y = random.uniform(a_4, a_1)
            z = random.uniform(a_4, a_1)
            roll = 0
            pitch = 0
            yaw = 0
            quat = quaternion_from_euler(roll, pitch, yaw)
            pose_data.pose.position.x = x
            pose_data.pose.position.y = y
            pose_data.pose.position.z = z
            pose_data.pose.orientation.x = quat[0]
            pose_data.pose.orientation.y = quat[1]
            pose_data.pose.orientation.z = quat[2]
            pose_data.pose.orientation.w = quat[3]
            pose_list.append(pose_data)
        for i in range(len(pose_list)):
            model_request = SetModelStateRequest()
            model_request.model_state = pose_list[i]
            self.gazebo_client(model_request)
            loop.sleep()
            
       

    def home_move_totyu(self):
        loop = rospy.Rate(100)
        com = 0
        pose_list = []
        a_1 = -12
        a_4 = -15
    
        for i in range(len(self.occulution_object.occuluder_name)):
            pose_data = ModelState()
            pose_data.model_name =  self.occulution_object.occuluder_name[i] 
            x = random.uniform(a_4, a_1)
            y = random.uniform(a_4, a_1)
            z = random.uniform(a_4, a_1)
            roll = 0
            pitch = 0
            yaw = 0
            quat = quaternion_from_euler(roll, pitch, yaw)
            pose_data.pose.position.x = x
            pose_data.pose.position.y = y
            pose_data.pose.position.z = z
            pose_data.pose.orientation.x = quat[0]
            pose_data.pose.orientation.y = quat[1]
            pose_data.pose.orientation.z = quat[2]
            pose_data.pose.orientation.w = quat[3]
            pose_list.append(pose_data)

        for i in range(len(self.occulution_object.occuludy_name)):
            pose_data = ModelState()
            pose_data.model_name = self.occulution_object.occuludy_name[i]
            x = random.uniform(a_4, a_1)
            y = random.uniform(a_4, a_1)
            z = random.uniform(a_4, a_1)
            roll = 0
            pitch = 0
            yaw = 0
            quat = quaternion_from_euler(roll, pitch, yaw)
            pose_data.pose.position.x = x
            pose_data.pose.position.y = y
            pose_data.pose.position.z = z
            pose_data.pose.orientation.x = quat[0]
            pose_data.pose.orientation.y = quat[1]
            pose_data.pose.orientation.z = quat[2]
            pose_data.pose.orientation.w = quat[3]
            pose_list.append(pose_data)
        
        for i in range(len(pose_list)):
            loop.sleep()
            model_request = SetModelStateRequest()
            model_request.model_state = pose_list[i]
            self.gazebo_client(model_request)
        rospy.sleep(0.2)
        
    
    def execute(self):
        self.home_move()
        # self.decide_occuluder()
        # self.object_move()
        self.run_service()

    def run_service(self):
        service = rospy.Service("tsuchida_oculution_service", object_occuluder_service, self.service_callback)
        

    def service_callback(self, req):
        self.home_move_totyu()
        self.decide_occuluder()
        
        if req.box_is_set:
            self.box_move()
        else:
            self.remove_box()
        # rospy.sleep(0.3)
        rospy.sleep(0.1)
        self.object_move()
        rospy.sleep(0.9)
        # rospy.set_param("record_is_ok", True)
        response = object_occuluder_serviceResponse()
        response.output = self.occulution_object
        return response
            
        # self.object_move("HV8")
    def box_move(self):
        pose_data = ModelState()
        pose_data.model_name = "object_box"
        pose_data.pose.position.x = 0
        pose_data.pose.position.y = 0
        self.box_z = self.z_coordinamte + 0.015
        pose_data.pose.position.z = self.box_z
        roll = 0
        pitch = 0
        yaw = 0
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose_data.pose.orientation.x = quat[0]
        pose_data.pose.orientation.y = quat[1]
        pose_data.pose.orientation.z = quat[2]
        pose_data.pose.orientation.w = quat[3]
        model_request = SetModelStateRequest()
        model_request.model_state = pose_data
        loop = rospy.Rate(40)
        loop.sleep()
        self.gazebo_client(model_request)
            

    
    def remove_box(self):
        pose_list = []
        loop = rospy.Rate(8)
        pose_data = ModelState()
        pose_data.model_name = "object_box"
        pose_data.pose.position.x = -12
        pose_data.pose.position.y = -15
        self.box_z = self.z_coordinamte
        pose_data.pose.position.z = self.box_z
        
        roll = 0
        pitch = 0
        yaw = 0
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose_data.pose.orientation.x = quat[0]
        pose_data.pose.orientation.y = quat[1]
        pose_data.pose.orientation.z = quat[2]
        pose_data.pose.orientation.w = quat[3]
        model_request = SetModelStateRequest()
        model_request.model_state = pose_data
        for i in range(2):
            self.gazebo_client(model_request)
            rospy.sleep(0.1)


    
    def decide_occuluder(self):
        self.occulution_object = object_kiriwake()
        # array_ocu_1 = list(range(len(self.occuluder_array)))
        array_ocu_1 = self.occuluder_array
        self.occulution_object.count = self.kaisuu
        self.kaisuu = self.kaisuu + 1
        random.shuffle(array_ocu_1)
        array_ocudy_2 = self.occuludy_array
        random.shuffle(array_ocudy_2)

        for i in range(random.randint(3, len(self.occuluder_array) - 1)):
            self.occulution_object.occuluder_name.append(str(self.model_name) + "_" + str(array_ocu_1[i]))
            # self.occulution_object.occuluder_name.append("ffe")
            self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_" + str(array_ocu_1[i]))
            self.occulution_object.occuluder_instance_numbers.append(array_ocu_1[i])

        for i in range(random.randint(0, len(self.occulution_object.occuluder_name) - 1)):
            self.occulution_object.occuludy_name.append(str(self.model_name) + "_" + str(array_ocudy_2[i]))
            self.occulution_object.occuludy_instance_number.append(array_ocudy_2[i])

   

if __name__=='__main__':
    rospy.init_node('random_state')
    model_name = rospy.get_param("~model_name", "HV8")
    ano = annotation_environment(model_name)
    rospy.spin()
    