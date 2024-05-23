from origami_eye_in_hand_interfaces.srv import DetectMarkers                             

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import cv2.aruco as aruco
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
MARKER_IDS = [12]

bridge = CvBridge()
ros_img = None
marker_flag = False
markers_set = False

# camera matrix vars
MARKER_SIZE_IN_MM = 38
CAM_CALIBRATION_MTX = np.array([[432.469055, 0.000000, 423.653994],
        [0.000000, 432.438101, 243.947633],
        [0.000000, 0.000000, 1.000000]])
CAM_DISTORTION_VALS = np.array([-0.044099, 0.029807, -0.000185, 0.000981, 0.000000]) 

class Marker:
  def __init__(self, id):
    self.id             = id
    self.center         = None
    self.depth          = None
    self.th             = None
    self.index          = None
    self.corners_list   = None
    self.desired_center = None
    self.error          = None
    
    self.past_centers       = []
    self.past_corners_list  = []

down = 1.0                # Image downsampling factor before skeletonization

new_goal_pos = None

class MarkerDetector(Node):

    def __init__(self):
        super().__init__('marker_detector')
        self.marker_array = []
        for id in MARKER_IDS:
            self.marker_array.append(Marker(id))

        self.srv = self.create_service(DetectMarkers, 'detect_markers', self.detect_markers_callback)
        self.publisher_compressed_color_img = self.create_publisher(CompressedImage, 'compressed_color_img',0)

    def detect_markers_callback(self, request, response):
        current_frame = bridge.imgmsg_to_cv2(request.color_img)
        
        self.assign_marker_desired_centers_and_errors(request)
        self.marker_pose(request.color_img, request.norm, request)

        # if markers have been set, update current frame w/ returned img
        if marker_flag:
            current_frame = bridge.imgmsg_to_cv2(ros_img)
        
        bgr_img = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("RGB", bgr_img)
        cv2.setMouseCallback('RGB', self.click_event)
        cv2.waitKey(1)
        
        # desired markers
        response.markers            = Float64MultiArray()
        response.desired_centers    = Float64MultiArray()
        
        # Convert pose to rosmsg & publish
        # Only if markers in frame, return marker response
        if (marker_flag):
            response.markers.data  = []
            response.desired_centers.data = []
            
            for marker in self.marker_array:
                # add x, y, depth of marker center
                response.markers.data.extend(marker.center)
                response.markers.data.append(marker.depth)
                
                # add x, y, depth of top left marker corner
                response.markers.data.extend(marker.corners_list[0])
                response.markers.data.append(marker.depth)
                
                # send back desired centers (if none have changed)
                response.desired_centers.data.append(request.desired_centers.data[0])
                response.desired_centers.data.append(request.desired_centers.data[1])
                response.desired_centers.data.append(request.desired_centers.data[2])
                response.desired_centers.data.append(request.desired_centers.data[3])
                
                # add desired_centers
                response.desired_centers.data.extend(marker.desired_center) # TODO potentially unnecessary?
                
        # publish for video converter
        self.publish_compressed_color_img(bgr_img)
        
        return response

    def publish_compressed_color_img(self, bgr_img):
        # slightly modified from http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        #### Create CompressedImage ####
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', bgr_img)[1]).tostring()
        
        self.publisher_compressed_color_img.publish(msg)

    def assign_marker_desired_centers_and_errors(self, request):
        for i in range(len(self.marker_array)):
            self.marker_array[i].desired_center = request.desired_centers.data[i:i+2]
            self.marker_array[i].error          = request.error.data[i:i+4]

    def click_event(self, event, x, y, flags, params):
        global new_goal_pos

        if event == cv2.EVENT_LBUTTONDOWN:
            new_goal_pos = [x,y]
            
            # logic for only one marker, changes desired center
            if new_goal_pos != None:
                self.marker_array[0].desired_center = new_goal_pos

    def marker_pose(self, img_msg, xz_cam_norm, request):
        global bridge, ros_img, marker_flag, MARKER_SIZE_IN_MM, CAM_CALIBRATION_MTX, CAM_DISTORTION_VALS
        cv_img = bridge.imgmsg_to_cv2(img_msg)

        # add corner list to markers
        corners, ids, rejectedImgPoints = self.detect_markers(cv_img)
        
        self.create_corner_list(corners, ids)
        try:
            for marker in self.marker_array:
                    # find marker center
                    marker_center_x = (marker.corners_list[0][0] + marker.corners_list[1][0] + marker.corners_list[2][0] + marker.corners_list[3][0])/4
                    marker_center_y = (marker.corners_list[0][1] + marker.corners_list[1][1] + marker.corners_list[2][1] + marker.corners_list[3][1])/4
                    
                    # set marker center
                    marker.center = [marker_center_x/down, marker_center_y/down]
                    marker.past_centers.append(marker.center)
                    
                    # find th
                    marker.th = math.atan2((marker.corners_list[2][1]/down - marker.corners_list[3][1]/down),(marker.corners_list[2][0]/down - marker.corners_list[3][0]/down))

                    # find depth
                    rvec_ee , tvec_ee, _        = aruco.estimatePoseSingleMarkers(corners[marker.index], MARKER_SIZE_IN_MM, CAM_CALIBRATION_MTX, CAM_DISTORTION_VALS)
                    marker.depth                = tvec_ee[0][0][2]

                    # draw marker params
                    self.draw_marker(cv_img, marker, request)
                
            # Draw xz norm
            cv2.arrowedLine(cv_img, (10, 40), (10 + int(xz_cam_norm/1000), 40), (30,30,30))
            # Draw a box on detected markers
            cv_img = aruco.drawDetectedMarkers(cv_img, corners)
            # Convert back to ros img to publish
            ros_img = bridge.cv2_to_imgmsg(cv_img)
            
        except TypeError:
            print("Markers not found!!!")
        
    def detect_markers(self, cv_img):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        return detector.detectMarkers(gray)
    
    def draw_marker(self, cv_img, marker, request):
        marker_center_x = marker.center[0]
        marker_center_y = marker.center[1]
        
        self.draw_trajectories(cv_img, marker)
                
        # draw marker center and corner
        cv2.circle(cv_img, (int(marker_center_x), int(marker_center_y)), 7, [255,0,0], -1)
        cv2.circle(cv_img, (int(marker.corners_list[0][0]), int(marker.corners_list[0][1])), 7, [255,0,0], -1)
        
        # draw desired marker center and corner
        cv2.circle(cv_img, (int(marker.desired_center[0]), int(marker.desired_center[1])), 7, [255, 133, 26], -1)
        cv2.circle(cv_img, (int(request.desired_centers.data[2]), int(request.desired_centers.data[3])), 7, [255, 133, 26], -1)

    def draw_trajectories(self, cv_img, marker):
        if(len(marker.error) != 0):
            # if list lengths not same, append last element until they are (only for visualization purposes)
            if len(marker.past_centers) > len(marker.past_corners_list):
                while len(marker.past_centers) > len(marker.past_corners_list): 
                    marker.past_corners_list.append(marker.past_corners_list[-1])

            # draw past centers and corners trajectories
            for i in range(len(marker.past_centers) - 1):
                cv2.line(cv_img, (int(marker.past_centers[i][0]), int(marker.past_centers[i][1])), (int(marker.past_centers[i+1][0]), int(marker.past_centers[i+1][1])), [0, 255, 0], 3)
                cv2.line(cv_img, (int(marker.past_corners_list[i][0]), int(marker.past_corners_list[i][1])), (int(marker.past_corners_list[i+1][0]), int(marker.past_corners_list[i+1][1])), [0, 255, 0], 3)
        
    def create_corner_list(self, corners, ids):
        global marker_flag, markers_set
        
        id_list = []
        id_list.clear()
        
        # get ids in frame that are desired markers 
        if ids is not None:
            print("these are ids presort", ids)
            for marker in self.marker_array:
                marker.index = None
                for i in ids:
                    if int(i[0]) == marker.id:
                        id_list.append(int(i[0]))
                        marker.index = id_list.index(marker.id)
        else:
            for marker in self.marker_array:
                marker.index = None
            marker_flag = False
        
        # are markers present?
        if(len(id_list) == len(self.marker_array)):
            marker_flag = True
            markers_set = True
        else:
            marker_flag = False

        # if all markers are present, set the corner list for each marker.
        if marker_flag:
            for marker in self.marker_array:
                marker.corners_list = corners[marker.index].reshape(4,2)
                marker.past_corners_list.append(marker.corners_list[0])
                
def main(args=None):
    rclpy.init(args=args)

    marker_detector = MarkerDetector()

    rclpy.spin(marker_detector)

    rclpy.shutdown()

if __name__ == '__main__':
    main()