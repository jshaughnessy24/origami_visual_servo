#!/bin/sh

# Find bag name in .ROS/
export bag_path=$(find -L ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log -name "*.db3") && # && means that if any of them don't work, script stops there.

# export topics to csv
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'marker_coordinates');"            > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/marker_coordinates.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'marker_desired_coordinates');"    > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/marker_desired_coordinates.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'tendon_lengths');"                > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/tendon_lengths.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'motor_velocities');"              > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/motor_velocities.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'motor_desired_velocities');"      > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/motor_desired_velocities.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'motor_encoder_counts');"          > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/motor_encoder_counts.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'xz_cam_desired_velocity');"       > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/xz_cam_desired_velocity.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'xz_cam_desired_velocity_norm');"  > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/xz_cam_desired_velocity_norm.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'marker_coordinates_image_errors');"       > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/marker_coordinates_image_errors.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'marker_coordinates_image_errors_norm');"  > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/marker_coordinates_image_errors_norm.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'camera_interaction_matrix');"     > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/camera_interaction_matrix.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'robot_jacobian');"                > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/robot_jacobian.csv &&
sqlite3 -header -csv $bag_path "select * from messages where topic_id = (select id from topics where name = 'end_flag');"                > ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/end_flag.csv &&

# Convert to gif
ffmpeg -t 60 -i ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/output_video.mp4 -filter_complex "[0:v] split [a][b];[a] palettegen [p];[b][p] paletteuse" ~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/output_gif.gif
exit