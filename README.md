# origami_msgs

# origami_visual_servo

# origami_data_logging


# Data logger node
In title_to_data_type_dict, 
replace the current titles and data types with your own topic names and message data types.

In data_type_to_message_type_dict,
replace current data and message types with the message and data types used by your topics.

The data logger should be set up to receive messages from those topics.

To use image_to_video_converter.py, you must send messages that are compressed images. You can make a normal image and then call the compress function on it. In image_to_video_converter.py, you may set a frame rate.
You should also set the topic that you are listening from. 

 # Main application
In your f

 To use the main origami eih package,
 cd to ros2_ws.
 colcon build.
 source install/setup.bash
 (if necessary)
 Then use the following command in the linux terminal:
    bash src/projects/origami_arm/origami_eye_in_hand/scripts/origami_eih_exp.sh
Input an experiment number.

    