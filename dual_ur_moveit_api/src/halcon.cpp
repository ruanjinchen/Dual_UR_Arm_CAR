#include "../include/halcon.h"

// 打印矩阵
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
    Eigen::Matrix4d matrixLeft = poseToMatrix(poseLeft);
    Eigen::Matrix4d matrixRight = poseToMatrix(poseRight);
    Eigen::Matrix4d composedMatrix = matrixLeft * matrixRight;
    return matrixToPose(composedMatrix);
}

// 以ZYX的形式从欧拉角转齐次矩阵
Eigen::Matrix4d createHomogeneousMatrix(double x, double y, double z, double rx_deg, double ry_deg, double rz_deg) {
    double rx_rad = rx_deg * M_PI / 180.0;
    double ry_rad = ry_deg * M_PI / 180.0;
    double rz_rad = rz_deg * M_PI / 180.0;

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(rz_rad, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(ry_rad, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(rx_rad, Eigen::Vector3d::UnitX());

    Eigen::Matrix4d homogeneous_matrix = Eigen::Matrix4d::Identity();
    homogeneous_matrix.block<3, 3>(0, 0) = rotation_matrix;
    homogeneous_matrix.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

    return homogeneous_matrix;
}

// 创建位姿 以XYZ旋转次序
Pose create_pose(double x, double y, double z, double rx_deg, double ry_deg, double rz_deg) {
    Pose pose;
    double rx_rad = rx_deg * M_PI / 180.0;
    double ry_rad = ry_deg * M_PI / 180.0;
    double rz_rad = rz_deg * M_PI / 180.0;

    pose.rotation = Eigen::AngleAxisd(rz_rad, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(ry_rad, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(rx_rad, Eigen::Vector3d::UnitX());

    pose.translation = Eigen::Vector3d(x, y, z);

    return pose;
}

Pose convertZYXtoXYZ(const Pose& originalPose) {
    Eigen::Vector3d eulerZYX = originalPose.rotation.toRotationMatrix().eulerAngles(2, 1, 0);

    Eigen::Quaterniond newRotation;
    newRotation = Eigen::AngleAxisd(eulerZYX[0], Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(eulerZYX[1], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(eulerZYX[2], Eigen::Vector3d::UnitX());

    Pose newPose;
    newPose.translation = originalPose.translation;
    newPose.rotation = newRotation;

    return newPose;
}

// 位姿反向
Pose pose_invert(const Pose &pose) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = pose.rotation.toRotationMatrix();
    matrix.block<3, 1>(0, 3) = pose.translation;

    Eigen::Matrix4d inverse_matrix = matrix.inverse();

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
