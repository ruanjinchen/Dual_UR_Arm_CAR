#include <memory>
#include <chrono> //循环时间头文件
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp> //标记消息头文件
#include <moveit/move_group_interface/move_group_interface.h> // MoveIt MoveGroup 接口头文件
#include <geometry_msgs/msg/pose_stamped.hpp> // 末端姿态消息头文件


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  // Create a Map marker publisher
  auto const marker_publisher = node->create_publisher<visualization_msgs::msg::Marker>("marker_topic", 10);
  // Create the MoveIt MoveGroup Interface 
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator"); //change
  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.0647;
    msg.orientation.y = -0.212;
    msg.orientation.z = 0.339;
    msg.orientation.w = 0;
    msg.position.x = 0;
    msg.position.y = 1;
    msg.position.z = 0;
    return msg;
  }();
  
  move_group_interface.setPoseTarget(target_pose);

  // 循环频率为50Hz
  rclcpp::Rate rate(50);

  // 获取当前末端姿态
  while(rclcpp::ok){
    geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose("wrist_3_joint");
    // 输出当前末端姿态
    RCLCPP_INFO(logger,"Current End Effector Pose:");
    RCLCPP_INFO(logger,"Position (x, y, z): %f, %f, %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    RCLCPP_INFO(logger,"Orientation (x, y, z, w): %f, %f, %f, %f", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    rate.sleep();
  }
  // // Create a plan to that target pose
  // auto const [success, plan] = [&move_group_interface]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();
  // // Execute the plan
  // if(success) {
  //   move_group_interface.execute(plan);
  // } else {
  //   RCLCPP_ERROR(logger, "Planing failed!");
  // }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
  }


