#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using namespace std::placeholders;
using MsgString = example_interfaces::msg::String;
using FloatArray = example_interfaces::msg::Float64MultiArray;
class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node>node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
        // 1. Subscription Gripper
        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper", 10, std::bind(&Commander::openGripperCallback, this, _1));

        // 2. Subscription Arm Named Target
        arm_named_sub_ = node_->create_subscription<MsgString>(
            "arm_named_target", 10, std::bind(&Commander::armNamedCallback, this, _1));

        // 3. Subscription Arm Joint Target
        arm_joint_sub_ = node_->create_subscription<FloatArray>(
            "arm_joint_target", 10, std::bind(&Commander::armJointCallback, this, _1));

        pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1));    
    }
    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false )
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();
        if (!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
       else
       {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);

            if (fraction == 1) {
                arm_->execute(trajectory);
            }
       }
    } 
    void armNamedCallback(const MsgString::SharedPtr msg)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(msg->data);
        planAndExecute(arm_);
    }

    void armJointCallback(const FloatArray::SharedPtr msg)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(msg->data);
        planAndExecute(arm_);
    }
    void poseCmdCallback(const PoseCmd &msg)
    {
        goToPoseTarget(msg.x, msg.y, msg.z, msg. roll, msg.pitch, msg.yaw, msg.cartesian_path);
    }
private:
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;

        if (success)
        {
            interface->execute(plan);
        }

    }
    void openGripperCallback(const Bool &msg)
    {
         if (msg.data) {
             gripper_->setStartStateToCurrentState();
             gripper_->setNamedTarget("gripper_open");
             planAndExecute(gripper_);
          }
         else {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("gripper_closed");
            planAndExecute(gripper_);
         }
    }
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<MsgString>::SharedPtr arm_named_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr arm_joint_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}