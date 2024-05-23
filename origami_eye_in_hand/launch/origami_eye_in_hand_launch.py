from launch import LaunchDescription, actions, event_handlers, events
from launch_ros.actions import Node

def generate_launch_description():
    velocity_controller = Node(
            package="origami_eye_in_hand",
            executable="velocity_controller",
            name="velocity_controller",
        )
    
    return LaunchDescription([
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node", 
            name="realsense2_camera_node"
        ),
        Node(
            package="origami_data_logging",
            executable="marker_detector",
            name="marker_detector"
        ),
        Node(
            package="origami_eye_in_hand",
            executable="single_module_jacobian_server",
            name="single_module_jacobian_server"
        ),
        velocity_controller,
        Node(
            package="origami_msgs" ,
            executable="motor_controller" ,
            name="motor_controller"
        ),
        Node(
            package="origami_data_logging",
            executable="bag_data_logger" ,
            name="bag_data_logger",
        ),
        Node(
            package="origami_data_logging",
            executable="image_to_video_converter",
            name="image_to_video_converter"
        ),
        
        actions.RegisterEventHandler(
            event_handler=event_handlers.OnProcessExit(
                target_action=velocity_controller,
                on_exit=[
                    actions.LogInfo(
                        msg="Listener exited; tearing down entire system."),
                    actions.EmitEvent(
                        event=events.Shutdown())]))
    ])