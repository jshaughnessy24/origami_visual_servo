import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Bool
from datetime import datetime
from threading import Thread
from sensor_msgs.msg import Image

import rosbag2_py

class BagDataLogger(Node):
    def __init__(self):
        super().__init__('bag_data_logger')
        
        self.title_to_data_type_dict = {
            'marker_coordinates'                    :'std_msgs/msg/Float64MultiArray', # each marker has its own entry--split every 3
            'marker_desired_coordinates'            :'std_msgs/msg/Float64MultiArray',
            'tendon_lengths'                        :'std_msgs/msg/Float64MultiArray', # each motor has its own entry 
            'motor_velocities'                      :'std_msgs/msg/Float64MultiArray', # each motor has its own entry
            'motor_desired_velocities'              :'std_msgs/msg/Float64MultiArray', # each motor has its own entry
            'motor_encoder_counts'                  :'std_msgs/msg/Float64MultiArray', # each motor has its own entry--split every 4
            'xz_cam_desired_velocity'               :'std_msgs/msg/Float64MultiArray', # desired xz camera velocity
            'xz_cam_desired_velocity_norm'          :'std_msgs/msg/Float64', 
            'marker_coordinates_image_errors'       :'std_msgs/msg/Float64MultiArray',
            'marker_coordinates_image_errors_norm'  :'std_msgs/msg/Float64',
            'camera_interaction_matrix'             :'std_msgs/msg/Float64MultiArray',
            'robot_jacobian'                        :'std_msgs/msg/Float64MultiArray',
            'end_flag'                              :'std_msgs/msg/Bool',
            }

        self.data_type_to_msg_type_dict = {  # couldn't find a function for it on my own, but it'd probably be like this anyways.
            'std_msgs/msg/Float64MultiArray' : Float64MultiArray,
            'std_msgs/msg/Float64'           : Float64,
            'sensor_msgs/msg/Image'          : Image,
            'std_msgs/msg/Bool'              : Bool
        }

        # create writer for bag
        self.writer = rosbag2_py.SequentialWriter()
        now = datetime.now()            #system date for bag name
        
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='src/projects/origami_arm/exp_log/' + str(now).replace(" ","_"),
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        self.title_to_data_dict = {}
        
        self.title_to_callback_func_dict = {}
        self.another_arr = []
        
        for title in self.title_to_data_type_dict.keys():
            data_type = self.title_to_data_type_dict[title]
            
            # create topic in bag
            bag_topic_info = rosbag2_py._storage.TopicMetadata(
            name=title,
            type=data_type,
            serialization_format='cdr')
            self.writer.create_topic(bag_topic_info)
            
            # subscribe to topic
            self.subscription = self.create_subscription(
            msg_type=self.data_type_to_msg_type_dict[data_type],
            topic=title,
            callback=self.get_callback_func(title),
            qos_profile=0)
            
            self.another_arr.append(self.subscription)
            
    def get_callback_func(self, title):
        return (lambda msg : self.title_to_data_dict.update({title: msg}))

def main(args=None):
    rclpy.init(args=args)
    bag_data_logger = BagDataLogger()

    spin_thread = Thread(target=rclpy.spin, args=(bag_data_logger,))
    spin_thread.start()

    rate = bag_data_logger.create_rate(10)

    try:
        while rclpy.ok():
            now = bag_data_logger.get_clock().now().nanoseconds
            
            # write to bag
            for title in bag_data_logger.title_to_data_type_dict.keys():
                if(title in bag_data_logger.title_to_data_dict.keys()):
                    bag_data_logger.writer.write(
                    title,
                    ("%s" % bag_data_logger.title_to_data_dict[title].data.tolist() if bag_data_logger.title_to_data_type_dict[title] == 'std_msgs/msg/Float64MultiArray' else "%s" % bag_data_logger.title_to_data_dict[title].data), 
                    now)
            
            if ('end_flag' in bag_data_logger.title_to_data_dict.keys() and bag_data_logger.title_to_data_dict['end_flag'].data):
                break
            
            rate.sleep()
    except KeyboardInterrupt:
        pass

    print("logger shutdown.")
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()