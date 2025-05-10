from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbslam3',
            executable='stereo-inertial',
            name='orbslam3_stereo_inertial',
            output='screen',
            remappings=[
                ('/cam0/image_raw', '/a15/camera/left/image_raw'),
                ('/cam1/image_raw', '/a15/camera/right/image_raw'),
                ('/imu0', '/a15/imu/data')
            ],
            arguments=[
                '/root/aqua_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt',
                '/root/aqua_ws/src/orbslam3_ros2/config/stereo-inertial/Aqua_front_stereo.yaml',
                'false',
                'false'
            ]
        )
    ])
