#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("rclcpp logger");
  RCLCPP_INFO(logger, "Running node...");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  RCLCPP_INFO(logger, "Creating move group interface...");
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "robocol_arm_group");
  RCLCPP_INFO(logger, "Move group interface created.");

  // Set a target Pose
  RCLCPP_INFO(logger, "Setting target pose...");
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.4;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  RCLCPP_INFO(logger, "Target pose set.");
  // Create a plan to that target pose
  RCLCPP_INFO(logger, "Creating plan...");
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  RCLCPP_INFO(logger, "Plan created.");
  // Execute the plan
  RCLCPP_INFO(logger, "Executing plan...");
  if(success) {
    move_group_interface.execute(plan);
  RCLCPP_INFO(logger, "Plan executed.");
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Set a target Pose
  RCLCPP_INFO(logger, "Setting target pose...");
  auto const target_pose_1 = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 1.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.4;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose_1);
  RCLCPP_INFO(logger, "Target pose set.");
  // Create a plan to that target pose
  RCLCPP_INFO(logger, "Creating plan...");
  auto const [success1, plan1] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  RCLCPP_INFO(logger, "Plan created.");
  // Execute the plan
  RCLCPP_INFO(logger, "Executing plan...");
  if(success1) {
    move_group_interface.execute(plan1);
  RCLCPP_INFO(logger, "Plan executed.");
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Set a target Pose
  RCLCPP_INFO(logger, "Setting target pose...");
  auto const target_pose_2 = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 2.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.4;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose_2);
  RCLCPP_INFO(logger, "Target pose set.");
  // Create a plan to that target pose
  RCLCPP_INFO(logger, "Creating plan...");
  auto const [success2, plan2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  RCLCPP_INFO(logger, "Plan created.");
  // Execute the plan
  RCLCPP_INFO(logger, "Executing plan...");
  if(success2) {
    move_group_interface.execute(plan2);
  RCLCPP_INFO(logger, "Plan executed.");
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // // Set a target Pose
  // RCLCPP_INFO(logger, "Setting target pose...");
  // auto const target_pose_3 = []{
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = 1.5;
  //   msg.position.x = -0.28;
  //   msg.position.y = -0.2;
  //   msg.position.z = 0.7;
  //   return msg;
  // }();
  // move_group_interface.setPoseTarget(target_pose_3);
  // RCLCPP_INFO(logger, "Target pose set.");
  // // Create a plan to that target pose
  // RCLCPP_INFO(logger, "Creating plan...");
  // auto const [success3, plan3] = [&move_group_interface]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();
  // RCLCPP_INFO(logger, "Plan created.");
  // // Execute the plan
  // RCLCPP_INFO(logger, "Executing plan...");
  // if(success3) {
  //   move_group_interface.execute(plan3);
  // RCLCPP_INFO(logger, "Plan executed.");
  // } else {
  //   RCLCPP_ERROR(logger, "Planing failed!");
  // }

  // // Set a target Pose
  // RCLCPP_INFO(logger, "Setting target pose...");
  // auto const target_pose_4 = []{
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = 0.9;
  //   msg.position.x = 0.28;
  //   msg.position.y = -0.1;
  //   msg.position.z = 0.4;
  //   return msg;
  // }();
  // move_group_interface.setPoseTarget(target_pose_4);
  // RCLCPP_INFO(logger, "Target pose set.");
  // // Create a plan to that target pose
  // RCLCPP_INFO(logger, "Creating plan...");
  // auto const [success4, plan4] = [&move_group_interface]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();
  // RCLCPP_INFO(logger, "Plan created.");
  // // Execute the plan
  // RCLCPP_INFO(logger, "Executing plan...");
  // if(success4) {
  //   move_group_interface.execute(plan4);
  // RCLCPP_INFO(logger, "Plan executed.");
  // } else {
  //   RCLCPP_ERROR(logger, "Planing failed!");
  // }
  

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}