#!/usr/bin/env python3


# Computes and returns the current value of Jacobian, given the cable lengths 
# also takes in the distance from each tendon to center of endplate, often 40mm
# Output: 1x9 vector with Jacobian elements to be shaped as 3x3

# Taken from single_module_jacobian_server.py in vs_control in merlab_ws

from __future__ import print_function
from os import path
import rclpy
from rclpy.node import Node
#from std_msgs.msg import Float64MultiArray
from origami_eye_in_hand_interfaces.srv import SingleModuleJacobian
# from vs_control.srv import SingleModuleJacobian, SingleModuleJacobianResponse
import dill
HOME = path.expanduser('~')
print(HOME)
# print(os.environ.get("ROS_PACKAGE_PATH "))
# jacobian_binary_path=HOME+"/mer_lab/ros_ws/src/projects/origami_arm/vs_control/scripts/single_module_Jacobian"
jacobian_binary_path=HOME+"/ros2_ws/src/origami_eye_in_hand/origami_eye_in_hand/single_module_Jacobian"

def avoid_singularity(l2, l3):
    if abs(l2 - l3) < 0.1:
        return l2 + 0.2
    else:
        return l2



# def single_module_jacobian_server():
#     rospy.init_node('single_module_jacobian_server')

#     s =  rospy.Service('single_module_jacobian', SingleModuleJacobian, handle_single_module_jacobian)

#     print("Ready to calculate Jacobian for single module.")
#     rospy.spin()

class SingleModuleJacobianServer(Node):
    def __init__(self):
        super().__init__('single_module_jacobian_server')
        self.srv = self.create_service(SingleModuleJacobian, 'single_module_jacobian', self.handle_single_module_jacobian)

    def handle_single_module_jacobian(self, request, response):
        global jacobian_binary_path
        lambda_jacobian =  dill.load(open(jacobian_binary_path, "rb"))
        # print("Single module cable length --> Jacobian")
        l2 =  avoid_singularity(request.l2,  request.l3)
        jacobian_matrix = lambda_jacobian(request.l1, l2, request.l3, request.d)
        print(jacobian_matrix)

        array =  [jacobian_matrix[0][0], jacobian_matrix[0][1], jacobian_matrix[0][2],\
                jacobian_matrix[1][0], jacobian_matrix[1][1], jacobian_matrix[1][2],\
                jacobian_matrix[2][0], jacobian_matrix[2][1], jacobian_matrix[2][2],\
                jacobian_matrix[3][0], jacobian_matrix[3][1], jacobian_matrix[3][2],\
                jacobian_matrix[4][0], jacobian_matrix[4][1], jacobian_matrix[4][2],\
                jacobian_matrix[5][0], jacobian_matrix[5][1], jacobian_matrix[5][2]]
    
        response.jv = array #actual jacobian
        response.step = 3   #to navigate, since its a 3x3 (actually 3x6 now) jacobian
        print("I've returned a response!")
        return response

def main(args=None):
    rclpy.init(args=args)
    single_module_jacobian_server = SingleModuleJacobianServer()
    print("Ready to calculate Jacobian for single module.")
    rclpy.spin(single_module_jacobian_server)
    rclpy.shutdown()

if __name__ == "__main__":
    # single_module_jacobian_server()
    main()
