#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <string>
#include <jsoncpp/json/json.h>
#include "../include/halcon.h"



int main(int argc, char * argv[])
{
  // 初始化ROS节点
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 创建logger
  auto const logger = rclcpp::get_logger("Knock!Knock!");

  // 创建Moveit2 MoveGroupInterface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "right");

  // auto CurrentPose = move_group_interface.getCurrentPose("right_tool0");
  // RCLCPP_INFO(node->get_logger(), "Pose Position: [%.3f, %.3f, %.3f], Orientation: [%.3f, %.3f, %.3f, %.3f]",
  //               CurrentPose.pose.position.x, CurrentPose.pose.position.y, CurrentPose.pose.position.z,
  //               CurrentPose.pose.orientation.x, CurrentPose.pose.orientation.y, CurrentPose.pose.orientation.z, CurrentPose.pose.orientation.w);

  // auto A = move_group_interface.getEndEffectorLink();
  // RCLCPP_INFO(logger,"EndEffectorLink:%s",A.c_str());

  // auto C = move_group_interface.getPoseReferenceFrame();
  // RCLCPP_INFO(logger,"PoseReferenceFrame:%s",C.c_str());

  // rx = rx * M_PI / 180.0;
  // ry = ry * M_PI / 180.0;
  // rz = rz * M_PI / 180.0;
  std::string message1 = "--------------------PoseFinal--------------------";
  std::string message2 = "-------------------------------------------------";

  std::ifstream json_file("/home/x/ur/src/dual_ur_moveit_api/src/pose.json"); // 位姿的json路径
    if (!json_file.is_open()) {
        std::cerr << "Failed to open file" << std::endl;
        return 1;
    }

    // 创建Json::Value对象并使用Json::Reader解析文件
    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(json_file, root)) {
        std::cerr << "Failed to parse JSON" << std::endl;
        return 1;
    }

    // 读取位姿数据
    double x = root["X"].asDouble();
    double y = root["Y"].asDouble();
    double z = root["Z"].asDouble();
    double rx = root["RX"].asDouble();
    double ry = root["RY"].asDouble();
    double rz = root["RZ"].asDouble();

    // 输出读取的值
    std::cout << "Message: " << message2 << std::endl;
    std::cout << "X: " << x << ", Y: " << y << ", Z: " << z << std::endl;
    std::cout << "RX: " << rx << ", RY: " << ry << ", RZ: " << rz << std::endl;

  Pose ToolInBasePose = create_pose(-0.33596, -0.14983, -0.03449, -107.05, -88.72, 18.08); //抓取拍摄的位置
  Pose ObjInCamPose = create_pose(0, 0, 0.342, 0, 0, -90); //物体在相机坐标系下的坐标


  Pose ToolInCamPose = create_pose(0.0308694, 0.0480492, -0.0826701, 0.367353, 0.674097, 359.408); //TCP在相机坐标系下的姿态
  Pose GripperInToolPose = create_pose(0, 0, -0.16, 0, 0, 90); //抓取中心与TCP之间的偏移
  Pose CamInToolPose, CamInBasePose, ObjInBasePose, PoseFinal, PoseFinal_XYZ;
  // 位姿组合
  CamInToolPose = pose_invert(ToolInCamPose);
  CamInBasePose = pose_compose(ToolInBasePose, CamInToolPose);
  ObjInBasePose = pose_compose(CamInBasePose, ObjInCamPose);
  PoseFinal = pose_compose(ObjInBasePose, GripperInToolPose);
  PoseFinal_XYZ = convertZYXtoXYZ(PoseFinal);
  std::cout << "Message: " << message1 << std::endl;
  printPose(PoseFinal);
  std::cout << "Message: " << message2 << std::endl;
  printPose(PoseFinal_XYZ);

  // //测试用
  // // 欧拉角
  // double roll = 30.0;   // 绕X轴的旋转
  // double pitch = 45.0;  // 绕Y轴的旋转
  // double yaw = 60.0;    // 绕Z轴的旋转
  // // 欧拉角转四元数
  // Quaternion q = EulerToQuaternion(roll, pitch, yaw);
  // 输出结果
  // std::cout << "Quaternion:" << std::endl;
  // std::cout << "w: " << q.w << std::endl;
  // std::cout << "x: " << q.x << std::endl;
  // std::cout << "y: " << q.y << std::endl;
  // std::cout << "z: " << q.z << std::endl;

  // 设定目标
  auto const target_pose_stamped = [&]{
    geometry_msgs::msg::PoseStamped msg_stamped;
    msg_stamped.header.frame_id = "right_base"; // 设置参考坐标系
    msg_stamped.header.stamp = node->get_clock()->now();  // 设置当前时间戳
    msg_stamped.pose.position.x = PoseFinal.translation.x();
    msg_stamped.pose.position.y = PoseFinal.translation.y();
    msg_stamped.pose.position.z = PoseFinal.translation.z();
    msg_stamped.pose.orientation.x = PoseFinal.rotation.x();
    msg_stamped.pose.orientation.y = PoseFinal.rotation.y();
    msg_stamped.pose.orientation.z = PoseFinal.rotation.z();
    msg_stamped.pose.orientation.w = PoseFinal.rotation.w();
    return msg_stamped;
  }();

  move_group_interface.setPoseTarget(target_pose_stamped);

  // 创建任务
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // 执行任务
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}