#pragma once

#include "Eigen/Eigen"
#include <cmath>
#include <cstdint>
#include <chrono>
#include <iostream>

#include "common_header.h"

#define PI 3.14159265358979323846
#define EPSILON 1e-13
#define RAD_TO_DEG(r) ((r)*180.0/PI)
#define DEG_TO_RAD(d) ((d)*PI/180.0)

// 四元数类定义
class Quaternion
{
public:
    double w, x, y, z;

    Quaternion() { w = 1; x = 0; y = 0; z = 0; }
    // 构造函数
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // 四元数点积
    double dot(const Quaternion& q) const
    {
        return w * q.w + x * q.x + y * q.y + z * q.z;
    }

    // 四元数乘法
    Quaternion operator*(const Quaternion& q) const
    {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    // 四元数加法
    Quaternion operator+(const Quaternion& q) const
    {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    // 四元数减法
    Quaternion operator-(const Quaternion& q) const
    {
        return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
    }
    Quaternion operator-() const
    {
        return Quaternion(-w, -x, -y, -z);
    }

    // 四元数的共轭
    Quaternion conjugate() const
    {
        return Quaternion(w, -x, -y, -z);
    }

    // 四元数乘以标量
    Quaternion operator*(const double &scalar) const
    {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    // 四元数除以标量
    Quaternion operator/(const double &scalar) const
    {
        return (*this) * (1.0 / scalar);
    }

    // 归一化四元数
    Quaternion normalize() const
    {
        double norm = sqrt(w * w + x * x + y * y + z * z);
        return Quaternion(w / norm, x / norm, y / norm, z / norm);
    }
};


namespace Utils
{
void euler_ABC_to_matrix(const double &A, const double &B, const double &C, Eigen::Matrix3d &matrix);
void euler_RPY_to_matrix(const double &roll, const double &pitch, const double &yaw, Eigen::Matrix3d &matrix);

void matrix_to_eular_ABC(const Eigen::Matrix3d &matrix, double &A, double &B, double &C);
void matrix_to_eular_RPY(const Eigen::Matrix3d &matrix, double &roll, double &pitch, double &yaw);

void euler_ABC_to_quaternion(const double &A, const double &B, const double &C, Quaternion &quat);
void quaternion_to_euler_ABC(const Quaternion &quat, double &A, double &B, double &C);

void plerp(const CartesianPosition &position_start, const CartesianPosition &position_end, const double &t, CartesianPosition &position_t);
void slerp(const Quaternion &quat_start, const Quaternion &quat_end, const double &t, Quaternion &quat_t);

Eigen::Matrix4d pose_to_matrix(const CartesianPose &pose);
CartesianPose matrix_to_pose(const Eigen::Matrix4d &mat);

void print_matrix(const Eigen::Matrix4d &mat);
}

namespace TimeDealUtils
{
    uint64_t get_timestamp();
    uint64_t get_time_difference(uint64_t start, uint64_t end);
    std::string timestamp_to_string(uint64_t timestamp);
}