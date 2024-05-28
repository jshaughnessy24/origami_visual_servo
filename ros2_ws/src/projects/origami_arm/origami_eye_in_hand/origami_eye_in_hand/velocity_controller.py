import array
from origami_eye_in_hand_interfaces.srv import DetectMarkers, SingleModuleJacobian                           # CHANGE
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread
from std_msgs.msg import Float64MultiArray, Bool
from std_msgs.msg import Float64
import numpy as np
import math
from enum import Enum

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        
        # publishes desired velocities
        self.publisher_marker_coordinates =         self.create_publisher(Float64MultiArray, 'marker_coordinates', 0)
        self.publisher_marker_desired_coordinates = self.create_publisher(Float64MultiArray, 'marker_desired_coordinates', 0)
        self.publisher_motor_desired_velocities =   self.create_publisher(Float64MultiArray, 'motor_desired_velocities', 0)
        self.publisher_xz_cam_desired_velocity =    self.create_publisher(Float64MultiArray, 'xz_cam_desired_velocity', 0)
        self.publisher_xz_cam_desired_velocity_norm =           self.create_publisher(Float64, 'xz_cam_desired_velocity_norm', 0)
        self.publisher_marker_coordinates_image_errors =        self.create_publisher(Float64MultiArray, 'marker_coordinates_image_errors', 0)
        self.publisher_marker_coordinates_image_errors_norm =   self.create_publisher(Float64, 'marker_coordinates_image_errors_norm', 0)
        self.publisher_camera_interaction_matrix =  self.create_publisher(Float64MultiArray, 'camera_interaction_matrix', 0)
        self.publisher_robot_jacobian =             self.create_publisher(Float64MultiArray, 'robot_jacobian', 0)
        
        self.publisher_end_flag =                   self.create_publisher(Bool, 'end_flag', 0)

        self.end_flag = False

        # subscribes to get rgb frames and current cable lengths
        self.subscription_rgb = self.create_subscription(Image, "/color/image_rect_raw", self.rgb_frame_callback, 0)
        self.subscription_length = self.create_subscription(Image, "/tendon_lengths", self.tendon_lengths_callback, 0)

        self.br_rgb = CvBridge()

        # creates markers and jacobian client
        self.cli_markers = self.create_client(DetectMarkers, 'detect_markers')       # CHANGE
        self.cli_jacobian = self.create_client(SingleModuleJacobian, 'single_module_jacobian')

        # sets up marker vals
        self.req_markers = DetectMarkers.Request() 
        self.markers = None

        # sets up jacobian vals
        self.req_jacobian = SingleModuleJacobian.Request()
        self.req_jacobian.d = 40.0      # 40 mm distance from center to each tendon
        self.req_jacobian.l1 = 150.0    # initial length of each tendon
        self.req_jacobian.l2 = 150.0
        self.req_jacobian.l3 = 150.0
        
        self.jacobian = None

        # flags vars
        self.hasReceivedColorImgFlag = False

        # TODO INITIALIZE TO CENTERS TO DEFAULT AND CHANGE DURING RUNTIME
        # AKA GENERALIZE FOR MULTIPLE MARKERS
        self.m1_center_desired = [695.25, 257.25]
        self.m1_corner_desired = [669.0, 229.0]
        self.marker_centers_desired = [self.m1_center_desired, self.m1_corner_desired]

        self.tendon_mm_per_rev = 7 * math.pi    
        self.req_markers.desired_centers = Float64MultiArray()
        self.req_markers.desired_centers.data = [self.m1_center_desired[0],self.m1_center_desired[1], self.m1_corner_desired[0], self.m1_corner_desired[1]]

        self.req_markers.error = Float64MultiArray()
        self.req_markers.norm = 1

        # camera intrinsics
        self.focal_length_x = 430.3659973144531 
        self.focal_length_y = 429.81927490234375 
        self.principle_pt_x = 426.06097412109375
        self.principle_pt_y = 244.31988525390625
        self.pixel_dim_ratio = 4/3 
        
    def send_marker_request(self):
        """
        Sends request for aruco marker coords.
    
        Return: 
            DetectMarkers.Response(): Response containing marker info
        """
        return self.cli_markers.call(self.req_markers)
    
    def send_jacobian_request(self):
        """
        Sends request for robot jacobian.
    
        Return: 
            SingleModuleJacobian.Response(): Response containing robot jacobian
        """
        return self.cli_jacobian.call(self.req_jacobian)

    def rgb_frame_callback(self, rgb_frame):
        """
        Updates rgb frame var self.req_markers.color_img
    
        Args:
            rgb_frame (sensor_msgs/Image): RGB camera frame
        """
        self.req_markers.color_img = rgb_frame
        self.hasReceivedColorImgFlag = True
    
    def tendon_lengths_callback(self, tendon_lengths): #TODO
        """
        Updates tendon length vars self.req_jacobian.l1, ...l2, ...l3
    
        Args:
            rgb_frame (sensor_msgs/Image): depth camera frame
        """
        self.req_jacobian.l1 = tendon_lengths.data[0]
        self.req_jacobian.l2 = tendon_lengths.data[1]
        self.req_jacobian.l3 = tendon_lengths.data[2]

    def find_image_frame_error(self, markers):
        """
        Finds image frame error.
    
        Return: 
            int[]: Array of coordinate errors (pixels) for each marker
        """
        m1_x_err = self.marker_centers_desired[0][0] - self.markers[0][0]
        m1_y_err = -(self.marker_centers_desired[0][1] - self.markers[0][1])
        
        m1_corner_x_err = self.marker_centers_desired[1][0] - self.markers[1][0]
        m1_corner_y_err = -(self.marker_centers_desired[1][1] - self.markers[1][1])
        
        return [m1_x_err, m1_y_err, m1_corner_x_err, m1_corner_y_err]  # TODO

    def find_normalized_coordinates(self, markers):
        """
        Finds normalized coordinates.
    
        Args:
            markers (int[][]): 2d array of pt features for visual servoing
        Return: 
            int[]: Array of normalized coordinates [centerx, centery, cornerx, cornery]
        """
        return [(markers[0][0] - self.principle_pt_x)/(self.focal_length_x * self.pixel_dim_ratio), -(markers[0][1] - self.principle_pt_y) / (self.focal_length_y), (markers[1][0] - self.principle_pt_x)/(self.focal_length_x * self.pixel_dim_ratio), -(markers[1][1] - self.principle_pt_y) / (self.focal_length_y)]
    
    def compute_velocities(self, marker_errors, robot_jacobian):
        gain = 1 / 50

        normalized_coords = self.find_normalized_coordinates(self.markers)
        
        # gets motor velocities by conversion (instantaneous img vel) -> (instantaneous camera vel) -> (tendon length vel) -> (motor vel)
        
        xyz_cam_vel    = self.normalized_coords_to_xyz_cam_vel(marker_errors, normalized_coords)
        # scale z cam vel (way too large)
        xyz_cam_vel[2] /= 2
        
        tendon_length_vel   = self.xyz_cam_vel_to_tendon_length_vel(robot_jacobian, xyz_cam_vel, gain)
        motor_vel           = self.get_rpm_from_tendon_vel(tendon_length_vel)
        
        # set norm for visualization
        self.req_markers.norm = int(np.linalg.norm(xyz_cam_vel))
       
        # publish xz vel and norm and cam/robot jacobian
        msg = Float64MultiArray()
        
        msg.data = xyz_cam_vel.tolist()
        self.publisher_xz_cam_desired_velocity.publish(msg)
        
        msg.data = np.array(robot_jacobian).flatten().tolist()
        self.publisher_robot_jacobian.publish(msg)#TODO
        
        msg = Float64()
        msg.data = float(self.req_markers.norm)
        self.publisher_xz_cam_desired_velocity_norm.publish(msg)
        
        return np.array(motor_vel).transpose() 

    def xz_cam_vel_to_tendon_length_vel(self, robot_jacobian, xz_cam_vel, gain):
        pinv_jacobian = pinv(robot_jacobian)

        # get tendon vel in mm / unit time
        tendon_length_vel = gain * np.matmul(pinv_jacobian, np.array([xz_cam_vel[0], 0, xz_cam_vel[1]]))
        return tendon_length_vel
    
    def xyz_cam_vel_to_tendon_length_vel(self, robot_jacobian, xyzalpha_cam_vel, gain):
        pinv_jacobian = pinv(robot_jacobian)
        
        # get tendon vel in mm / unit time
        tendon_length_vel = gain * np.matmul(pinv_jacobian, np.array([xyzalpha_cam_vel[0], xyzalpha_cam_vel[1], xyzalpha_cam_vel[2]]))
        return tendon_length_vel

    def get_cam_interaction_matrix(self, marker_errors, camera_interaction_matrix, normalized_coords):
        # creates large interaction matrix from camera interaction matrices for each marker
        for i in range(int(len(marker_errors) / 2)):
            depth = None
            if i == 0:
                depth = self.markers[0][2]
            else:
                depth = self.markers[1][2] # remnant from if more than one marker
            if(camera_interaction_matrix is None):
                camera_interaction_matrix = np.array([[-1/depth,   normalized_coords[i]/depth], 
                                                      [0,          normalized_coords[i+1]/depth]])
            else:
                camera_interaction_matrix = np.concatenate((camera_interaction_matrix, 
                                                       np.array([[-1/depth,   normalized_coords[2*i]/depth], 
                                                                 [0,          normalized_coords[2*i+1]/depth]])), axis=0)
                                                                 
        return camera_interaction_matrix   
        
    def get_cam_interaction_matrix_xyz(self, marker_errors, camera_interaction_matrix, normalized_coords):
        # creates large interaction matrix from camera interaction matrices for each marker
        for i in range(int(len(marker_errors) / 2)):
            depth = None
            if i == 0:
                depth = self.markers[0][2]
            else:
                depth = self.markers[1][2]
                
            if(camera_interaction_matrix is None):
                camera_interaction_matrix = np.array([[-1/depth,    0,          normalized_coords[2*i]/depth  ], 
                                                      [0,           -1/depth,   normalized_coords[2*i+1]/depth ]])
            else:
                camera_interaction_matrix = np.concatenate((camera_interaction_matrix, 
                        np.array([[-1/depth,    0,          normalized_coords[2*i]/depth     ], 
                                  [0,           -1/depth,   normalized_coords[2*i+1]/depth   ]])), 
                            axis=0)
                
        return camera_interaction_matrix
                
    def normalized_coords_to_xz_cam_vel(self, marker_errors, normalized_coords):
        camera_interaction_matrix = None
        
        camera_interaction_matrix = self.get_cam_interaction_matrix(marker_errors, camera_interaction_matrix, normalized_coords)
        pinv_camera_interaction_matrix = pinv(camera_interaction_matrix)
        
        #publish current interaction matrix
        msg = Float64MultiArray()
        msg.data = camera_interaction_matrix.flatten().tolist()
        self.publisher_camera_interaction_matrix.publish(msg)#TODO
        
        #get camera vel
        return -1 * np.matmul(pinv_camera_interaction_matrix, np.array(marker_errors).transpose())
    
    def normalized_coords_to_xyz_cam_vel(self, marker_errors, normalized_coords):
        camera_interaction_matrix = None
        camera_interaction_matrix = self.get_cam_interaction_matrix_xyz(marker_errors, camera_interaction_matrix, normalized_coords)
        
        pinv_camera_interaction_matrix = pinv(camera_interaction_matrix)
        
        # publish current interaction matrix
        msg = Float64MultiArray()
        msg.data = camera_interaction_matrix.flatten().tolist()
        self.publisher_camera_interaction_matrix.publish(msg)#TODO
        
        #get camera vel
        return -1 * np.matmul(pinv_camera_interaction_matrix, np.array(marker_errors).transpose())
    
    def get_rpm_from_tendon_vel(self, tendon_length_vel):
        return np.divide(tendon_length_vel, self.tendon_mm_per_rev)
    
    
def main(args=None):
    rclpy.init(args=args)

    velocity_controller = VelocityController()
    
    spin_thread = Thread(target=rclpy.spin, args=(velocity_controller,), daemon=True)
    spin_thread.start()

    rate = velocity_controller.create_rate(10)

    try:
        while rclpy.ok() and not velocity_controller.end_flag:
            # services available?
            while not velocity_controller.cli_markers.wait_for_service(timeout_sec=1.0) or not velocity_controller.cli_jacobian.wait_for_service(timeout_sec=1.0):
                velocity_controller.get_logger().info('services not available, waiting again...')
            
            # if imgs available, send marker req 
            if velocity_controller.hasReceivedColorImgFlag:
                update_markers_and_desired_centers(velocity_controller)

            # get jacobian, and compute velocities
            if velocity_controller.markers is not None and velocity_controller.markers != []:
                update_jacobian(velocity_controller)
                
                # determine image frame errors
                velocity_controller.error = velocity_controller.find_image_frame_error(velocity_controller.markers)
                velocity_controller.req_markers.error = Float64MultiArray()
                velocity_controller.req_markers.error.data = velocity_controller.error

                # determine motor velocities
                velocity_controller.computed_velocities = velocity_controller.compute_velocities(velocity_controller.error, velocity_controller.jacobian)

            
                # publish desired velocities
                desired_velocities = Float64MultiArray()
                desired_velocities.data = velocity_controller.computed_velocities.tolist()
                velocity_controller.publisher_motor_desired_velocities.publish(desired_velocities)
                
                # publish image errors
                velocity_controller.publisher_marker_coordinates_image_errors.publish(velocity_controller.req_markers.error)
                
                # publish image norm
                marker_coordinates_image_errors_norm = Float64()
                marker_coordinates_image_errors_norm.data = np.linalg.norm(np.array(velocity_controller.req_markers.error.data))
                velocity_controller.publisher_marker_coordinates_image_errors_norm.publish(marker_coordinates_image_errors_norm)
                
                # has program reached end state?
                if (np.linalg.norm(desired_velocities.data) < 1): # if tendon vel norm < 1, end program
                    velocity_controller.end_flag = True
                    
                # publish end flag
                end_flag_msg = Bool()
                end_flag_msg.data = velocity_controller.end_flag
                velocity_controller.publisher_end_flag.publish(end_flag_msg)
                
                
                rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    spin_thread.join()
    
def update_jacobian(velocity_controller):
    unformatted_jacobian = velocity_controller.send_jacobian_request().jv
    
    velocity_controller.jacobian = [unformatted_jacobian[0:3], unformatted_jacobian[3:6], unformatted_jacobian[6:9]]

def update_markers_and_desired_centers(velocity_controller):
    # set desired centers
    received_marker_req = velocity_controller.send_marker_request()
    received_markers            = received_marker_req.markers.data
    received_desired_centers    = received_marker_req.desired_centers.data
    
    # publish actual and desired centers
    velocity_controller.publisher_marker_coordinates.publish(received_marker_req.markers)
    velocity_controller.publisher_marker_desired_coordinates.publish(received_marker_req.desired_centers)
    
    # format received markers/centers
    if received_desired_centers != array.array('d'):
        velocity_controller.markers = []
        velocity_controller.marker_centers_desired = []
        
        for i in range(int(len(received_markers)/3)):
            velocity_controller.markers.append(received_markers[3*i:3*i+3])
            velocity_controller.marker_centers_desired.append(received_desired_centers[2*i:2*i+2])
            
        velocity_controller.req_markers.desired_centers.data = [velocity_controller.marker_centers_desired[0][0], velocity_controller.marker_centers_desired[0][1], velocity_controller.marker_centers_desired[1][0], velocity_controller.marker_centers_desired[1][1]] #TODO 0 won't work for more than 1 marker

def pinv(matrix):
    return np.linalg.pinv(matrix)

if __name__ == '__main__':
    main()