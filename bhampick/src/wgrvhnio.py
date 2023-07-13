#!/usr/bin/env python3

import rospy
import struct
from sensor_msgs.msg import PointCloud2
from cv_connector.srv import CV_srv, CV_srvRequest
from rospy_message_converter import message_converter
from std_msgs.msg import String

dict = {1:{"bbox":"a", "center":[1.1, 1.2], "name":"c"},
        2:{"bbox":"d", "center":[2.1, 2.2], "name":"f"},
        3:{"bbox":"g", "center":[3.1, 3.2], "name":"i"},
        4:{"bbox":"j", "center":[4.1, 4.2], "name":"l"},
        5:{"bbox":"m", "center":[5.1, 5.2], "name":"o"},
        6:{"bbox":"p", "center":[5.3, 5.4], "name":"r"}}

def get_target_center(thedict, target):
    for a, b in thedict.items():
        print(a, b)
        # if b['name'] == target:
            # return b['center'][0], b['center'][1]
            
get_target_center(dict, 'c')
        
# x, y = get_target_center(dict, 'c')
# print(x, y)

# class haiyaaaa:
#     def __init__(self):
#         self.rospy.init_node("cv_comp_track")
#         self.client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
#         self.srv = CV_srvRequest()
        
#     def get_dict(self, target = "target"):
#         try:
#             rospy.wait_for_service('/CV_connect/req_cv')
#             response = self.client(self.srv)
#             result = String(response.result)
#             mydict = message_converter.convert_ros_message_to_dictionary(result)
#             mypointcloud = response.pointcloud
            
#         except rospy.ServiceException as e:
#             rospy.logerr("Failed to call service: %s", str(e))
#             exit(1)
            
#         for a, b in mydict.items():
#             if b['name'] == target:
#                 x = b['center'][0]
#                 y = b['center'][1]
                
#         assert isinstance(mypointcloud, PointCloud2)
#         index = (y * mypointcloud.row_step) + (x * mypointcloud.point_step)
#         mypoint = struct.unpack_from('fff', mypointcloud.data, offset=index)
#         return mypoint
        
# if __name__ == "__main__":
#     k = haiyaaaa()
#     rospy.spin()