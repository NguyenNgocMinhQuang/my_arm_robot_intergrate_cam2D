#!/usr/bin/env python3
"""
pick_place_state_machine.py

State Machine Pick & Place thu cong (khong dung MoveIt Task Constructor).
Dieu khien canh tay/gripper qua MoveItPy (PlanningComponent), nhan pose vat
the tu topic /detected_object_pose (do node ArUco publish, trong
camera_optical_frame), tu tra cuu TF de doi sang base_link, roi chay qua
10 state theo dung yeu cau de bai.

Ghi chu ve "Cartesian":
MoveItPy (Python binding cua MoveIt2) hien khong expose truc tiep
computeCartesianPath() nhu MoveGroupInterface ben C++. De giu code chay
duoc bang API chinh thuc, APPROACH/LIFT/LOWER duoc thuc hien bang cach
dat pose-goal (OMPL se tim duong ngan, gan nhu thang neu khong co vat
can) thay vi Cartesian path that su. Neu can duong thang tuyet doi, ban
co the:
  - Vet mot node/service rieng bang C++ (MoveGroupInterface::computeCartesianPath)
    va goi tu Python qua ROS service, hoac
  - Chia nho pose-goal thanh nhieu waypoint gan nhau.
"""

import math
from enum import Enum, auto
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (dang ky do_transform_pose cho PoseStamped)
import tf_transformations
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanningComponent
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener


ROBOT_CONFIG = MoveItConfigsBuilder(robot_name="my_robot", package_name="my_robot_moveit_config") \
    .robot_description_semantic("config/my_robot.srdf", {"name": "my_robot"}) \
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
    "planning_pipelines": {"pipeline_names": ["ompl"]},
    "plan_request_params": {
        "planning_attempts": 5,
        "planning_pipeline": "ompl",
        "max_velocity_scaling_factor": 0.5,
        "max_acceleration_scaling_factor": 0.5,
    },
    "ompl": {
        "planning_plugins": ["ompl_interface/OMPLPlanner"],
        "request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision",
        ],
        "response_adapters": [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ],
        "start_state_max_bounds_error": 0.1,
    },
}


class State(Enum):
    IDLE = auto()
    OPEN_GRIPPER = auto()
    MOVE_PRE_GRASP = auto()
    APPROACH = auto()
    CLOSE_GRIPPER = auto()
    LIFT = auto()
    MOVE_TO_PLACE = auto()
    LOWER = auto()
    OPEN_AND_DETACH = auto()
    RETREAT_HOME = auto()


class PickPlaceStateMachine(Node):
    def __init__(self):
        super().__init__("pick_place_state_machine")

        # ---------------- Tham so cau hinh ----------------
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eef_link", "tool_link")          # link gan vat the/EE cua nhom "arm"
        self.declare_parameter("gripper_open_state", "gripper_open")
        self.declare_parameter("gripper_closed_state", "gripper_closed")
        self.declare_parameter("home_state", "home")
        self.declare_parameter("pre_grasp_offset_z", 0.15)
        self.declare_parameter("approach_clearance_z", 0.02)     # (hien khong dung trong APPROACH,
                                                                   # giu lai de mo rong sau nay neu can)
        self.declare_parameter("gripper_tcp_offset_z", 0.1)                                                          
        self.declare_parameter("lift_height", 0.15)
        self.declare_parameter("box_size", 0.04)                 # khoi hop 4x4x4 cm
        self.declare_parameter("pose_stale_timeout", 1.0)        # (s) bo qua pose ArUco cu
                # ---- Hieu chuan gripper: width(q) = C + A*cos(q) + B*sin(q) ----
        # Cac gia tri nay lay tu ket qua chay calibrate_gripper.py, KHONG tuy chinh tay
        self.declare_parameter("gripper_calib_c", 0.01439)
        self.declare_parameter("gripper_calib_a", 0.05061)
        self.declare_parameter("gripper_calib_b", 0.03296)
        self.declare_parameter("gripper_joint_min", -0.7)
        self.declare_parameter("gripper_joint_max", 0.15)
        self.declare_parameter("grasp_compliance_margin", 0.003)  # (m) khep hut so voi box_size
        self.declare_parameter("gripper_visual_overhang", 0.013)  # (m) phan mesh nho ra ngoai collision box, do bang mat/thuoc
        # Vi tri dat (place pose), chinh lai theo ban lam viec thuc te cua ban
        self.declare_parameter("place_x", 0.0)
        self.declare_parameter("place_y", 0.2)
        self.declare_parameter("place_z", 0.05)

        self.base_frame_ = self.get_parameter("base_frame").value
        self.eef_link_ = self.get_parameter("eef_link").value
        self.gripper_open_state_ = self.get_parameter("gripper_open_state").value
        self.gripper_closed_state_ = self.get_parameter("gripper_closed_state").value
        self.home_state_ = self.get_parameter("home_state").value
        self.pre_grasp_offset_z_ = self.get_parameter("pre_grasp_offset_z").value
        self.approach_clearance_z_ = self.get_parameter("approach_clearance_z").value
        self.gripper_tcp_offset_z_ = self.get_parameter("gripper_tcp_offset_z").value
        self.lift_height_ = self.get_parameter("lift_height").value
        self.box_size_ = self.get_parameter("box_size").value
        self.pose_stale_timeout_ = self.get_parameter("pose_stale_timeout").value

                # ---- Doc tham so hieu chuan gripper ----
        self.gripper_calib_c_ = self.get_parameter("gripper_calib_c").value
        self.gripper_calib_a_ = self.get_parameter("gripper_calib_a").value
        self.gripper_calib_b_ = self.get_parameter("gripper_calib_b").value
        self.gripper_joint_min_ = self.get_parameter("gripper_joint_min").value
        self.gripper_joint_max_ = self.get_parameter("gripper_joint_max").value
        self.grasp_compliance_margin_ = self.get_parameter("grasp_compliance_margin").value
        self.gripper_visual_overhang_ = self.get_parameter("gripper_visual_overhang").value

        # Tinh san R, phi tu A, B (dung nhieu lan trong ham nghich dao)
        self.gripper_calib_r_ = math.hypot(self.gripper_calib_a_, self.gripper_calib_b_)
        self.gripper_calib_phi_ = math.atan2(self.gripper_calib_b_, self.gripper_calib_a_)

        self.place_pose_base_ = Pose()
        self.place_pose_base_.position.x = self.get_parameter("place_x").value
        self.place_pose_base_.position.y = self.get_parameter("place_y").value
        self.place_pose_base_.position.z = self.get_parameter("place_z").value
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(math.pi, 0.0, 0.0)
        self.place_pose_base_.orientation.x = qx
        self.place_pose_base_.orientation.y = qy
        self.place_pose_base_.orientation.z = qz
        self.place_pose_base_.orientation.w = qw

        # ---------------- MoveItPy ----------------
        self.robot_ = MoveItPy(node_name="moveit_py", config_dict=ROBOT_CONFIG)
        self.arm_: PlanningComponent = self.robot_.get_planning_component("arm")
        self.gripper_: PlanningComponent = self.robot_.get_planning_component("gripper")

        # ---------------- TF ----------------
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # ---------------- Planning scene service ----------------
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        self.apply_scene_client_ = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=self.client_cb_group
        )
        while not self.apply_scene_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /apply_planning_scene service...")

        # ---------------- Subscription pose vat the ----------------
        self.detected_pose_sub_ = self.create_subscription(
            PoseStamped, "/detected_object_pose", self.detected_pose_callback, 10
        )
        self.latest_object_pose_base_ = None   # Pose (da chuyen ve base_frame_)
        self.active_object_pose_base_ = None   # Pose duoc "chot" khi bat dau 1 chu trinh

        # ---------------- State machine ----------------
        self.state_ = State.IDLE
        # Gắn timer vào luồng riêng
        self.timer_ = self.create_timer(0.5, self.control_loop, callback_group=self.timer_cb_group)

    # ================= Nhan pose ArUco, chuyen ve base_frame =================
    def detected_pose_callback(self, msg: PoseStamped):
        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        age = (now - msg_time).nanoseconds / 1e9
        if age > self.pose_stale_timeout_:
            return  # pose qua cu, camera co the da mat marker

        try:
            transform = self.tf_buffer_.lookup_transform(
                self.base_frame_, msg.header.frame_id, rclpy.time.Time()
            )
        except Exception as ex:  # TransformException
            self.get_logger().warn(f"TF lookup {msg.header.frame_id} -> {self.base_frame_} failed: {ex}")
            return

        pose_base = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
        self.latest_object_pose_base_ = pose_base

    # ================= Vong lap state machine =================
    def control_loop(self):
        handler = {
            State.IDLE: self.handle_idle,
            State.OPEN_GRIPPER: self.handle_open_gripper,
            State.MOVE_PRE_GRASP: self.handle_move_pre_grasp,
            State.APPROACH: self.handle_approach,
            State.CLOSE_GRIPPER: self.handle_close_gripper,
            State.LIFT: self.handle_lift,
            State.MOVE_TO_PLACE: self.handle_move_to_place,
            State.LOWER: self.handle_lower,
            State.OPEN_AND_DETACH: self.handle_open_and_detach,
            State.RETREAT_HOME: self.handle_retreat_home,
        }[self.state_]

        self.get_logger().info(f"[STATE] {self.state_.name}")
        try:
            handler()
        except Exception as ex:
            self.get_logger().error(f"Loi trong state {self.state_.name}: {ex}. Quay ve IDLE.")
            self.state_ = State.IDLE

    # ---------------- Cac state ----------------
    def handle_idle(self):
        if self.latest_object_pose_base_ is not None:
            self.active_object_pose_base_ = self.latest_object_pose_base_
            self.latest_object_pose_base_ = None
            self.active_box_center_ = self.add_box_collision_object(self.active_object_pose_base_)
            self.state_ = State.OPEN_GRIPPER

    def handle_open_gripper(self):
        if self.move_to_named(self.gripper_, self.gripper_open_state_):
            self.state_ = State.MOVE_PRE_GRASP
        else:
            self.get_logger().warn("Khong the mo gripper, huy chu trinh.")
            self.remove_box_collision_object()
            self.state_ = State.IDLE

    def handle_move_pre_grasp(self):
        pre_grasp_pose = self.make_grasp_pose(
            self.active_box_center_, z_offset=self.pre_grasp_offset_z_
        )
        if self.move_to_pose(self.arm_, pre_grasp_pose):
            self.state_ = State.APPROACH
        else:
            self.get_logger().warn("Khong the plan toi pre-grasp, huy chu trinh.")
            self.remove_box_collision_object()
            self.state_ = State.IDLE

    def handle_approach(self):
        approach_pose = self.make_grasp_pose(
            self.active_box_center_, z_offset=self.gripper_tcp_offset_z_
        )
        if self.move_to_pose(self.arm_, approach_pose):
            self.state_ = State.CLOSE_GRIPPER
        else:
            self.get_logger().warn("Khong the plan toi approach, huy chu trinh.")
            self.remove_box_collision_object()
            self.state_ = State.IDLE

    def handle_close_gripper(self):
        # Khep gripper theo dung box_size_ (tru hao mot khoang compliance margin,
        # vi dong co buoc open-loop khong co phan hoi luc/dong dien).
        target_width = max(self.box_size_ - self.grasp_compliance_margin_ + 2 * self.gripper_visual_overhang_, 0.0)
        if self.move_gripper_to_width(target_width):
            self.attach_box_to_gripper()
            self.state_ = State.LIFT
        else:
            # Dong gripper that bai: KHONG gia vo da gap duoc vat, huy chu trinh
            self.get_logger().warn("Khong the dong gripper theo box_size, huy chu trinh.")
            self.remove_box_collision_object()
            self.state_ = State.RETREAT_HOME

    def handle_lift(self):
        lift_pose = self.make_grasp_pose(self.active_box_center_, z_offset=self.lift_height_)
        self.move_to_pose(self.arm_, lift_pose)  # co gang lift, du thanh cong hay khong van tiep tuc
        self.state_ = State.MOVE_TO_PLACE

    def handle_move_to_place(self):
        above_place = Pose()
        above_place.position.x = self.place_pose_base_.position.x
        above_place.position.y = self.place_pose_base_.position.y
        above_place.position.z = self.place_pose_base_.position.z + self.pre_grasp_offset_z_
        above_place.orientation = self.place_pose_base_.orientation
        if self.move_to_pose(self.arm_, above_place):
            self.state_ = State.LOWER
        else:
            self.state_ = State.RETREAT_HOME

    def handle_lower(self):
        self.move_to_pose(self.arm_, self.place_pose_base_)
        self.state_ = State.OPEN_AND_DETACH

    def handle_open_and_detach(self):
        self.move_to_named(self.gripper_, self.gripper_open_state_)
        self.detach_box_from_gripper()
        self.state_ = State.RETREAT_HOME

    def handle_retreat_home(self):
        self.move_to_named(self.arm_, self.home_state_)
        self.active_object_pose_base_ = None
        self.state_ = State.IDLE

        # ---------------- Helper gripper (width <-> joint angle) ----------------
    def gripper_angle_for_width(self, width: float) -> float:
        """
        Nghich dao cong thuc hieu chuan: width(q) = C + A*cos(q) + B*sin(q)
        -> tra ve goc joint gripper_control (rad) ung voi do mo mong muon (m).
        Ep ket qua ve trong gioi han joint that (gripper_joint_min_/max_).
        """
        ratio = (width - self.gripper_calib_c_) / self.gripper_calib_r_
        ratio = max(-1.0, min(1.0, ratio))
        theta = self.gripper_calib_phi_ - math.acos(ratio)
        theta_clamped = max(self.gripper_joint_min_, min(self.gripper_joint_max_, theta))
        if theta_clamped != theta:
            self.get_logger().warn(
                f"[GRIPPER] target_width={width:.4f} -> theta={theta:.4f} rad "
                f"NGOAI GIOI HAN, bi ep ve {theta_clamped:.4f}"
            )
        return theta_clamped
    
    def move_gripper_to_width(self, width: float) -> bool:
        """
        Dieu khien gripper khep/mo toi dung do mo mong muon (m), thay vi
        dung named target co dinh. Dung RobotState de set truc tiep joint
        chu dong 'gripper_control'; cac joint mimic (left1/left2/right2/right3...)
        se duoc MoveIt tu dong tinh theo, khong can set tay.
        """
        target_angle = self.gripper_angle_for_width(width)

        robot_state = RobotState(self.robot_.get_robot_model())
        robot_state.joint_positions = {"gripper_control": target_angle}

        self.gripper_.set_start_state_to_current_state()
        self.gripper_.set_goal_state(robot_state=robot_state)
        plan_result = self.gripper_.plan()
        if plan_result:
            self.robot_.execute(plan_result.trajectory, controllers=[])
            return True
        return False
    
    # ---------------- Helper chuyen dong ----------------
    def make_grasp_pose(self, object_pose: Pose, z_offset: float) -> Pose:
        pose = Pose()
        pose.position.x = object_pose.position.x
        pose.position.y = object_pose.position.y
        pose.position.z = object_pose.position.z + z_offset
        # Gripper huong thang xuong (roll = pi), giu nguyen yaw = 0.
        # Neu muon xoay gripper theo huong marker, doi yaw ben duoi bang yaw
        # trich xuat tu object_pose.orientation.
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(math.pi, 0.0, 0.0)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def move_to_pose(self, component: PlanningComponent, pose: Pose) -> bool:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame_
        pose_stamped.pose = pose

        component.set_start_state_to_current_state()
        component.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=self.eef_link_)
        plan_result = component.plan()
        if plan_result:
            self.robot_.execute(plan_result.trajectory, controllers=[])
            return True
        return False

    def move_to_named(self, component: PlanningComponent, name: str) -> bool:
        component.set_start_state_to_current_state()
        component.set_goal_state(configuration_name=name)
        plan_result = component.plan()
        if plan_result:
            self.robot_.execute(plan_result.trajectory, controllers=[])
            return True
        return False

    # ---------------- Planning scene: table + box ----------------
    def call_apply_planning_scene(self, scene: PlanningScene):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        # Ném lệnh đi và KHÔNG CHỜ phản hồi nữa, dứt điểm mọi loại Deadlock!
        future = self.apply_scene_client_.call_async(req)
        def _on_done(fut):
            try:
                res = fut.result()
                if not res.success:
                    self.get_logger().warn("apply_planning_scene tra ve success=False")
            except Exception as ex:
                self.get_logger().warn(f"apply_planning_scene loi: {ex}")

        future.add_done_callback(_on_done)
    def rotate_vector_by_quaternion(self, v, q):
        v = np.array(v, dtype=float)
        u = np.array([q[0], q[1], q[2]], dtype=float)
        s = q[3]
        cross1 = np.cross(u, v)
        cross2 = np.cross(u, cross1)
        return v + 2 * s * cross1 + 2 * cross2
    
    def add_box_collision_object(self, object_pose_base: Pose):
        q = (object_pose_base.orientation.x, object_pose_base.orientation.y,
            object_pose_base.orientation.z, object_pose_base.orientation.w)
        offset_local = (0.0, 0.0, -self.box_size_ / 2.0)  # luôn -Z cục bộ, mọi mặt dán
        offset_world = self.rotate_vector_by_quaternion(offset_local, q)

        box_center_pose = Pose()
        box_center_pose.position.x = object_pose_base.position.x + offset_world[0]
        box_center_pose.position.y = object_pose_base.position.y + offset_world[1]
        box_center_pose.position.z = object_pose_base.position.z + offset_world[2]
        box_center_pose.orientation.w = 1.0

        box = CollisionObject()
        box.header.frame_id = self.base_frame_
        box.id = "picked_box"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [self.box_size_] * 3
        box.primitives.append(primitive)
        box.primitive_poses.append(box_center_pose)
        box.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(box)
        self.call_apply_planning_scene(scene)

        return box_center_pose
    
    def remove_box_collision_object(self):
        box = CollisionObject()
        box.header.frame_id = self.base_frame_
        box.id = "picked_box"
        box.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(box)
        self.call_apply_planning_scene(scene)

    def attach_box_to_gripper(self):
        attached = AttachedCollisionObject()
        attached.link_name = self.eef_link_
        attached.object.id = "picked_box"
        attached.object.operation = CollisionObject.ADD
        # touch_links: cac link duoc phep cham vao box khi cam ma khong bao loi va cham
        attached.touch_links = ["tool_link", "hand_link", "gripper_base_link",
                                 "gripper_left1", "gripper_left2", "gripper_left3",
                                 "gripper_right1", "gripper_right2", "gripper_right3"]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        # Xoa box khoi world truoc do de tranh trung lap (attach se tu quan ly no)
        remove_from_world = CollisionObject()
        remove_from_world.id = "picked_box"
        remove_from_world.header.frame_id = self.base_frame_
        remove_from_world.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(remove_from_world)
        self.call_apply_planning_scene(scene)

    def detach_box_from_gripper(self):
        attached = AttachedCollisionObject()
        attached.link_name = self.eef_link_
        attached.object.id = "picked_box"
        attached.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        self.call_apply_planning_scene(scene)
        # Sau khi tha, xoa luon box khoi scene (coi nhu da dat xong, khong can theo doi nua)
        self.remove_box_collision_object()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceStateMachine()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
