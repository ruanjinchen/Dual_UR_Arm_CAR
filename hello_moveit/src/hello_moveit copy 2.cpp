#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

Eigen::Quaterniond q(1,0,0,0);
Eigen::Vector3d t(1,1,6);
 
Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
T.rotate(q);
T.pretranslate(t);
 
cout << T.matrix() << endl;
cout << T.inverse().matrix() << endl;
 
 
//向量
Eigen::Vector3d p1 = Eigen::Vector3d(0.5, 0, 0.2);
 
//欧拉矩阵定义 4×4
Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
T1.rotate(q1.toRotationMatrix());
T1.pretranslate(t1);
T2.rotate(q2.toRotationMatrix());
T2.pretranslate(t2);
// cout << T1.matrix() << endl;
// cout << T2.matrix() << endl;
 
//计算向量的旋转
p2 = T2 * T1.inverse() * p1;
cout << p2.transpose() << endl;