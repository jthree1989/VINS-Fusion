/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "utility.h"

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    //^ R0 represents rotation that ng2 = R0 * ng1, 
    //^ i.e. R0 is R_21(or orientation of g(ng1) in global frame(ng2))
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    //^ 这里R0只是让g旋转到了ng2,即重力对齐方向对齐，
    //^ 下面需要使沿ng2的旋转一致即yaw角一致：
    //^ 1) 计算R0的yaw角，即绕ng2的旋转；
    //^ 2) 左乘-yaw旋转矩阵，保持绕ng2的旋转不变；
    //^ 综上两步，保证R0即将重力方向对齐也保证研重力方向的旋转(yaw)不变
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
