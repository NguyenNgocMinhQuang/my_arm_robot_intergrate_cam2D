#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped
import tf_transformations


ROBOT_CONFIG = MoveItConfigsBuilder(robot_name="my_robot", package_name="my_robot_moveit_config")\
                                    .robot_description_semantic("config/my_robot.srdf", {"name": "my_robot"})\
                                    .to_dict()

ROBOT_CONFIG = { 
    **ROBOT_CONFIG,
    "planning_scene_monitor": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "planning_pipelines": {
            "pipeline_names": ["ompl"]
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 1.0,
            "max_acceleration_scaling_factor": 1.0
        },
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": ["default_planning_request_adapters/ResolveConstraintFrames",
                            "default_planning_request_adapters/ValidateWorkspaceBounds",
                            "default_planning_request_adapters/CheckStartStateBounds",
                            "default_planning_request_adapters/CheckStartStateCollision"],
            "response_adapters": ["default_planning_response_adapters/AddTimeOptimalParameterization",
                             "default_planning_response_adapters/ValidateSolution",
                             "default_planning_response_adapters/DisplayMotionPath"],
            "start_state_max_bounds_error": 0.1
        }
}

def main(args=None):
    rclpy.init(args=args)
    robot = MoveItPy(node_name="moveit_py", config_dict=ROBOT_CONFIG)
    arm: PlanningComponent = robot.get_planning_component("arm")
    gripper: PlanningComponent = robot.get_planning_component("gripper")

    # ----------------------- NAME -------------------------------------

    # arm.set_start_state_to_current_state()
    # arm.set_goal_state(configuration_name="pose_1")
    # plan_result = arm.plan()
    # if plan_result:
    #     robot.execute(plan_result.trajectory, controllers=[])

    # ----------------------- JOINTS -------------------------------------

    # robot_state = RobotState(robot.get_robot_model())

    # joint_values = {
    #     "joint1": 0.5,
    #     "joint2": 0.5,
    #     "joint3": 0.5,
    #     "joint4": 0.5,
    #     "joint5": 0.5,
    #     "joint6": 0.5,
    # }

    # robot_state.joint_positions = joint_values

    # arm.set_start_state_to_current_state()
    # arm.set_goal_state(robot_state=robot_state)
    # plan_result = arm.plan()
    # if plan_result:
    #     robot.execute(plan_result.trajectory, controllers=[])

    # ----------------------- POSE -------------------------------------

    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(3.14, 0.0, 0.0)
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.position.x = 0.0
    pose_goal.pose.position.y = -0.7
    pose_goal.pose.position.z = 0.4
    pose_goal.pose.orientation.x = q_x
    pose_goal.pose.orientation.y = q_y
    pose_goal.pose.orientation.z = q_z
    pose_goal.pose.orientation.w = q_w

    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool_link")
    plan_result = arm.plan()
    if plan_result:
        robot.execute(plan_result.trajectory, controllers=[])

    rclpy.shutdown()