from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Set environment variable for camera
        SetEnvironmentVariable(
            name="GSCAM_CONFIG",
            value=("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, "
                   "format=NV12, framerate=30/1 ! nvvidconv flip-method=2 ! nvvidconv ! "
                   "video/x-raw, width=224, height=224, format=BGRx ! videoconvert")
        ),

        # Joystick driver node
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{"dev_ff": "/dev/input/event2"}]
        ),

        # Joystick controller node
        Node(
            package="controller",
            executable="param_controller",
            name="param_controller_node",
            output="screen",
            parameters=[
                # Joystick Ratio Settings
                {"throttle_ratio": 0.6},
                {"steer_ratio": -1.0},
                # Joystick Channel Setting
                {"throttle_axis": 4},
                {"steer_axis": 0}
            ]
        ),

        # Actuator node
        Node(
            package="actuator",
            executable="pca9685_actuator",
            name="pca9685_actuator_node",
            output="screen",
            parameters=[
                # PWM Frequency and Bus Configuration
                {"bus_device": "/dev/i2c-7"},
                {"pwm_frequency": 60},

                # Throttle Configuration
                {"throttle_pwm_channel": 3},
                {"throttle_min_pulsewidth": 1000},
                {"throttle_max_pulsewidth": 2000},
                {"throttle_mid_pulsewidth": 1500},

                # Steer Configuration
                {"steer_pwm_channel": 0},
                {"steer_min_pulsewidth": 1200},
                {"steer_max_pulsewidth": 2000},
                {"steer_mid_pulsewidth": 1600},
            ]
        ),

        # Camera node
        Node(
            package="gscam",
            executable="gscam_node",
            name="camera",
            output="screen"
            # Optional: add parameters or remappings if needed
            # parameters=[{"camera_info_url": "package://localcam/calibrations/${NAME}.yaml"}],
            # remappings=[("camera/image_raw", "cam_name/image_raw")]
        )
    ])