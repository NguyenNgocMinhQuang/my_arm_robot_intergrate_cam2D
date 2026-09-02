"""
pick_and_place.launch.py

Khoi dong toan bo luong:
  1. Camera driver (usb_cam) publish /camera/image_raw + /camera/camera_info
  2. Static TF: base_link -> camera_link (hieu chuan ngoai vi THAT cua ban,
     PHAI do dac/hieu chuan lai cho dung robot cua ban)
  3. Static TF: camera_link -> camera_optical_frame (xoay chuan REP-103,
     KHONG can chinh sua)
  4. RViz + MoveGroup + fake/mock hardware controllers (tai su dung
     demo.launch.py co san trong my_robot_moveit_config)
  5. Node aruco_detector_node (my_robot_perception)
  6. Node pick_place_state_machine (my_robot_commander_py)

Cach chay:
  ros2 launch my_robot_bringup pick_and_place.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        "video_device", default_value="/dev/video0",
        description="Duong dan thiet bi webcam USB"
    )

    # ---- Cong tac chinh: false = mock (bay gio), true = robot that (tuong lai) ----
    use_real_hardware_arg = DeclareLaunchArgument(
        "use_real_hardware", default_value="false",
        description="true = chay tren phan cung that qua my_robot.launch.xml; "
                     "false = chay mock/fake hardware qua demo.launch.py"
    )
    use_real_hardware = LaunchConfiguration("use_real_hardware")
    # ---- 1. Camera driver ----
    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[{
            "video_device": LaunchConfiguration("video_device"),
            "image_width": 640,
            "image_height": 480,
            "pixel_format": "mjpeg2rgb",
            "camera_frame_id": "camera_optical_frame",
            "camera_name": "my_webcam",
            "camera_info_url": "file:///home/minh_quang/.ros/camera_info/my_webcam.yaml",
        }],
        remappings=[
            ("image_raw", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
    )

    # ---- 2. Static TF: base_link -> camera_link ----
    # !!! CHINH LAI x y z yaw pitch roll cho dung vi tri gan camera thuc te !!!
    static_tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_camera",
        arguments=[
            "--x", "0.1", "--y", "0.2", "--z", "0.2",
            "--yaw", "0.0", "--pitch", "1.5708", "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
        ],
    )

    # ---- 3. Static TF chuan: camera_link -> camera_optical_frame ----
    # Quaternion nay la quy uoc chuan REP-103, KHONG can doi.
    static_tf_camera_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera_to_optical",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
            "--frame-id", "camera_link",
            "--child-frame-id", "camera_optical_frame",
        ],
    )

        # ---- 4a. NHANH MOCK (mac dinh, dung bay gio) ----
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_robot_moveit_config"), "launch", "demo.launch.py"
            ])
        ),
        condition=UnlessCondition(use_real_hardware),
    )

    # ---- 4b. NHANH ROBOT THAT (dung khi co phan cung, bat bang use_real_hardware:=true) ----
    real_robot_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_robot_bringup"), "launch", "my_robot.launch.xml"
            ])
        ),
        launch_arguments={"use_mock_component": "false"}.items(),
        condition=IfCondition(use_real_hardware),
    )

    # ---- 5. Node ArUco detector ----
    aruco_detector_node = Node(
        package="my_robot_perception",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        output="screen",
        parameters=[{
            "image_topic": "/camera/image_raw",
            "camera_info_topic": "/camera/camera_info",
            "camera_optical_frame_id": "camera_optical_frame",
            "aruco_marker_frame_id": "aruco_marker_frame",
            "target_marker_id": 0,
            "marker_length": 0.04,
            "aruco_dictionary": "DICT_4X4_50",
        }],
    )

    # ---- 6. Node Pick & Place State Machine ----
    # Delay vai giay de move_group / planning_scene_monitor san sang truoc
    pick_place_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="my_robot_commander_py",
                executable="pick_place_state_machine",
                name="pick_place_state_machine",
                output="screen",
                parameters=[{
                    "base_frame": "base_link",
                    "eef_link": "tool_link",
                    "gripper_open_state": "gripper_open",
                    "gripper_closed_state": "gripper_closed",
                    "home_state": "home",
                    "pre_grasp_offset_z": 0.15,
                    "approach_clearance_z": 0.02,
                    "lift_height": 0.15,
                    "box_size": 0.04,
                    "place_x": 0.0,
                    "place_y": 0.2,
                    "place_z": 0.05,
                }],
            )
        ],
    )

    return LaunchDescription([
        video_device_arg,
        use_real_hardware_arg,
        usb_cam_node,
        static_tf_base_to_camera,
        static_tf_camera_to_optical,
        moveit_demo_launch,
        real_robot_launch,
        aruco_detector_node,
        pick_place_node,
    ])
