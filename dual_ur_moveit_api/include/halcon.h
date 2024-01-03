// halcon.h
#ifndef HALCON_H
#define HALCON_H

#include <Eigen/Dense>
#include <string>
#include <iostream>

// 四元数结构
struct Quaternion {
    double w, x, y, z;
};

// 位姿结构
struct Pose {
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
};

// 函数声明
void printMatrix(const Eigen::Matrix4d& matrix, const std::string& name);
Eigen::Matrix4d poseToMatrix(const Pose& pose);
Pose matrixToPose(const Eigen::Matrix4d& matrix);
Pose pose_compose(const Pose& poseLeft, const Pose& poseRight);
Eigen::Matrix4d createHomogeneousMatrix(double x, double y, double z, double rx_deg, double ry_deg, double rz_deg);
Pose create_pose(double x, double y, double z, double rx_deg, double ry_deg, double rz_deg);
Pose convertZYXtoXYZ(const Pose& originalPose);
Pose pose_invert(const Pose &pose);
double DegToRad(double degrees);
Quaternion EulerToQuaternion(double roll, double pitch, double yaw);
void printPose(const Pose& pose);

#endif // COMMON_H
