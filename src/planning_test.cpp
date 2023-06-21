#include <iostream>
#include <memory>
#include <string.h>
// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
// MoveIt libraries
#include <moveit/move_group_interface/move_group_interface.h>
using moveit::planning_interface::MoveGroupInterface;
// Placeholders
using std::placeholders::_1;

class PlanningNode : public rclcpp::Node {
  public :
    PlanningNode() : Node("planning_node") {
      // Create a ROS Subscription to robocol/arm_desired_pose with geometry_msgs::msg::Pose type message
      auto pose_topic = "robocol/arm_desired_pose";
      RCLCPP_INFO(this->get_logger(), "Subscribing to %s topic...", pose_topic);
      pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(pose_topic, 1, std::bind(&PlanningNode::set_target_pose, this, _1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to %s.", pose_topic);
      auto plan_topic = "robocol/arm/action";
      RCLCPP_INFO(this->get_logger(), "Subscribing to %s topic...", plan_topic);
      plan_subscription_ = this->create_subscription<std_msgs::msg::String>(plan_topic, 1, std::bind(&PlanningNode::set_action, this, _1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to %s.", plan_topic);
      // Create move_group ROS node
      move_group_node = rclcpp::Node::make_shared("move_group");
      // Create a ROS logger
      auto const logger = rclcpp::get_logger("rclcpp logger");
      // Create the MoveIt MoveGroup Interface
      RCLCPP_INFO(logger, "Creating move group interface...");
      move_group_ptr_ = new MoveGroupInterface(move_group_node, "robocol_arm_group");
      RCLCPP_INFO(logger, "Move group interface created.");
    }

    ~PlanningNode() {
      delete move_group_ptr_;
    }

  private :
    std::shared_ptr<rclcpp::Node> move_group_node;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_subscription_;
    geometry_msgs::msg::Pose target_pose;
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_;

    void set_action(const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      std::string plan_str = "plan";
      std::string execute_str = "execute";
      if (msg->data.c_str() == plan_str) {
        // Create a plan to that target pose
        RCLCPP_INFO(this->get_logger(), "Creating plan...");
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_ptr_->plan(msg));
        auto const [success, plan] = std::make_pair(ok, msg);
      } else if(msg->data.c_str() == execute_str) {
        // Create a plan to that target pose
        RCLCPP_INFO(this->get_logger(), "Creating plan...");
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_ptr_->plan(msg));
        auto const [success, plan] = std::make_pair(ok, msg);
        // Execute the plan
        RCLCPP_INFO(this->get_logger(), "Executing plan...");
        if(success) {
          move_group_ptr_->execute(plan);
          RCLCPP_INFO(this->get_logger(), "Plan executed.");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Planing failed!");
        }
      }
    }

    void set_target_pose(const geometry_msgs::msg::Pose pose) {
      RCLCPP_INFO(this->get_logger(), "I heard! %f", pose.orientation.x);
      target_pose = pose;
      RCLCPP_INFO(this->get_logger(), "Setting tareget pose...");
      move_group_ptr_->setPoseTarget(target_pose);
      RCLCPP_INFO(this->get_logger(), "Target pose set.");
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto planning_node = std::make_shared<PlanningNode>();
  rclcpp::spin(planning_node);
  rclcpp::shutdown();
  return 0;
}
