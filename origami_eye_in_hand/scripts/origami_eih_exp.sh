#!/bin/bash

# Written by Abhinav Ghandi
# Modified by Jennifer Shaughnessy for EIH control

# This bash script contains the experiment pipeline for simulation
# Launch the experiment with the reference loaded from YAML
# Run the experiment
# Once the end flag is raised, the experiment ends
# All the data is moved to an experiment folder
# Then data extraction and processing is performed automagically

# Input experiment number for folder
clear &&
read exp_no &&

# Launch
ros2 launch origami_eye_in_hand origami_eye_in_hand_launch.py && #instead launch your launch file.

# Extract logs
cd ~/mer_lab/ros2_ws/src/projects/origami_arm/origami_data_logging/scripts/ && #instead
./origami_eih_bag_to_csv.sh

# Plot data
matlab -nodisplay -r "plot_shape_grow_sim; exit" &&

# Organize data
# Create experiment folder
cd ~/Pictures/origami_eye_in_hand/test/ &&
mkdir $exp_no &&

# Move raw data
# bag file
export bag_path=$(find -L ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log -name "2*" -type d) && # added 2* is because the bag directories start with "2024-"etc.

mv $bag_path ~/Pictures/origami_eye_in_hand/test/$exp_no &&

# csv files
# Move all data to exps folder
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/marker_coordinates.csv            ~/Pictures/origami_eye_in_hand/test/$exp_no&&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/marker_desired_coordinates.csv    ~/Pictures/origami_eye_in_hand/test/$exp_no&& # > redirects the output into the .ros/error.csv file, 
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/tendon_lengths.csv                ~/Pictures/origami_eye_in_hand/test/$exp_no&&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/motor_velocities.csv              ~/Pictures/origami_eye_in_hand/test/$exp_no&&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/motor_desired_velocities.csv      ~/Pictures/origami_eye_in_hand/test/$exp_no&&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/motor_encoder_counts.csv          ~/Pictures/origami_eye_in_hand/test/$exp_no&&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/xz_cam_desired_velocity.csv       ~/Pictures/origami_eye_in_hand/test/$exp_no&& 
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/xz_cam_desired_velocity_norm.csv  ~/Pictures/origami_eye_in_hand/test/$exp_no&& 
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/marker_coordinates_image_errors.csv       ~/Pictures/origami_eye_in_hand/test/$exp_no&& 
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/marker_coordinates_image_errors_norm.csv  ~/Pictures/origami_eye_in_hand/test/$exp_no&& 
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/camera_interaction_matrix.csv     ~/Pictures/origami_eye_in_hand/test/$exp_no&& 
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/robot_jacobian.csv                ~/Pictures/origami_eye_in_hand/test/$exp_no&& 
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/end_flag.csv                ~/Pictures/origami_eye_in_hand/test/$exp_no&& 

# Move all the media
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/output_video.mp4 ~/Pictures/origami_eye_in_hand/test/$exp_no &&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/output_gif.gif ~/Pictures/origami_eye_in_hand/test/$exp_no &&

# Move plots
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/error_norm.png ~/Pictures/origami_eye_in_hand/test/$exp_no &&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/velocity.png ~/Pictures/origami_eye_in_hand/test/$exp_no &&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/image_velocity.png ~/Pictures/origami_eye_in_hand/test/$exp_no &&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/image_velocity_x.png ~/Pictures/origami_eye_in_hand/test/$exp_no &&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/image_velocity_y.png ~/Pictures/origami_eye_in_hand/test/$exp_no &&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/image_velocity_z.png ~/Pictures/origami_eye_in_hand/test/$exp_no &&
mv ~/mer_lab/ros2_ws/src/projects/origami_arm/exp_log/xyz_error_norm.png ~/Pictures/origami_eye_in_hand/test/$exp_no &&

echo "Processing completed, exiting"
exit 1