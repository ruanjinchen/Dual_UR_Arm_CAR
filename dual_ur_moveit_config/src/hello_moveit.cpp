#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <string>
#include <jsoncpp/json/json.h>


// 四元数结构
struct Quaternion {
    double w, x, y, z;
};

// 位姿结构
struct Pose {
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
};

//打印矩阵
void printMatrix(const Eigen::Matrix4d& matrix, const std::string& name) {
    std::cout << "Matrix " << name << ":" << std::endl;
    std::cout << matrix << std::endl << std::endl;
}

// 将位姿转换为齐次变换矩阵
Eigen::Matrix4d poseToMatrix(const Pose& pose) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = pose.rotation.toRotationMatrix();
    matrix.block<3, 1>(0, 3) = pose.translation;
    return matrix;
}

// 将齐次变换矩阵转换回位姿
Pose matrixToPose(const Eigen::Matrix4d& matrix) {
    Pose pose;
    pose.translation = matrix.block<3, 1>(0, 3);
    pose.rotation = Eigen::Quaterniond(matrix.block<3, 3>(0, 0));
    return pose;
}

// 齐次矩阵相乘
Pose pose_compose(const Pose& poseLeft, const Pose& poseRight) {
    // 将位姿转换为齐次变换矩阵
    Eigen::Matrix4d matrixLeft = poseToMatrix(poseLeft);
    Eigen::Matrix4d matrixRight = poseToMatrix(poseRight);
    
    // printMatrix(matrixLeft, "Left");
    // printMatrix(matrixRight, "Right");

    // 矩阵相乘
    Eigen::Matrix4d composedMatrix = matrixLeft * matrixRight;
    // printMatrix(composedMatrix, "Result");

    // 将结果转换回位姿
    return matrixToPose(composedMatrix);
}

//以ZYX的形式从欧拉角转齐次矩阵
Eigen::Matrix4d createHomogeneousMatrix(double x, double y, double z, double rx_deg, double ry_deg, double rz_deg) {
    // Convert angles from degrees to radians
    double rx_rad = rx_deg * M_PI / 180.0;
    double ry_rad = ry_deg * M_PI / 180.0;
    double rz_rad = rz_deg * M_PI / 180.0;

    // Create the rotation matrix using Eigen's AngleAxis objects for the XYZ rotation order
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(rz_rad, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(ry_rad, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(rx_rad, Eigen::Vector3d::UnitX());

    // Create the homogeneous transformation matrix
    Eigen::Matrix4d homogeneous_matrix = Eigen::Matrix4d::Identity();
    homogeneous_matrix.block<3, 3>(0, 0) = rotation_matrix;
    homogeneous_matrix.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

    return homogeneous_matrix;
}

// 创建位姿 以XYZ旋转次序
Pose create_pose(double x, double y, double z, double rx_deg, double ry_deg, double rz_deg) {
  Pose pose;
    // Convert angles from degrees to radians
    double rx_rad = rx_deg * M_PI / 180.0;
    double ry_rad = ry_deg * M_PI / 180.0;
    double rz_rad = rz_deg * M_PI / 180.0;

    // Create the rotation quaternion using Eigen's AngleAxis objects for the ZYX rotation order
    pose.rotation = Eigen::AngleAxisd(rz_rad, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(ry_rad, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(rx_rad, Eigen::Vector3d::UnitX());

    // Set the translation
    pose.translation = Eigen::Vector3d(x, y, z);

    return pose;
}

Pose convertZYXtoXYZ(const Pose& originalPose) {
    // 从原始位姿中提取ZYX顺序的欧拉角
    Eigen::Vector3d eulerZYX = originalPose.rotation.toRotationMatrix().eulerAngles(2, 1, 0);

    // 将欧拉角重新应用，使用XYZ顺序创建新的旋转四元数
    Eigen::Quaterniond newRotation;
    newRotation = Eigen::AngleAxisd(eulerZYX[0], Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(eulerZYX[1], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(eulerZYX[2], Eigen::Vector3d::UnitX());

    // 创建并返回新的位姿，保持原始位姿的平移不变
    Pose newPose;
    newPose.translation = originalPose.translation;
    newPose.rotation = newRotation;

    return newPose;
}

// 位姿反向
Pose pose_invert(const Pose &pose) {
    // 创建一个4x4的齐次变换矩阵
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = pose.rotation.toRotationMatrix();
    matrix.block<3, 1>(0, 3) = pose.translation;

    // 计算逆矩阵
    Eigen::Matrix4d inverse_matrix = matrix.inverse();

    // 从逆矩阵中提取平移和旋转
    Pose inverted_pose;
    inverted_pose.translation = inverse_matrix.block<3, 1>(0, 3);
    inverted_pose.rotation = Eigen::Quaterniond(inverse_matrix.block<3, 3>(0, 0));

    return inverted_pose;
}

// 角度转弧度
double DegToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

// 欧拉角转四元数
Quaternion EulerToQuaternion(double roll, double pitch, double yaw) {
    roll = DegToRad(roll);
    pitch = DegToRad(pitch);
    yaw = DegToRad(yaw);

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// 打印姿态
void printPose(const Pose& pose) {
    std::cout << "Translation: "
              << pose.translation.x() << ", "
              << pose.translation.y() << ", "
              << pose.translation.z() << std::endl;

    std::cout << "Rotation: "
              << pose.rotation.x() << ", "
              << pose.rotation.y() << ", "
              << pose.rotation.z() << ", "
              << pose.rotation.w() << std::endl;
}




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
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // auto CurrentPose = move_group_interface.getCurrentPose("tool0");
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

  std::ifstream json_file("/home/ruan/pose.json"); // 位姿的json路径
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
    msg_stamped.header.frame_id = "base"; // 设置参考坐标系
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









