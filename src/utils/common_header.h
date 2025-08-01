#pragma once

#include <string>
#include <vector>
#include <map>
#include "Eigen/Eigen"


struct CartesianPosition
{
    double x;
    double y;
    double z;

    CartesianPosition() :
        x(0.0), y(0.0), z(0.0)
    {
    }

    CartesianPosition(double x_, double y_, double z_) :
        x(x_), y(y_), z(z_)
    {
    }
};

struct CartesianOrientation
{
    double A;
    double B;
    double C;

    CartesianOrientation() :
        A(0.0), B(0.0), C(0.0)
    {
    }

    CartesianOrientation(double A_, double B_, double C_) :
        A(A_), B(B_), C(C_)
    {
    }
};

struct CartesianQuaternion
{
    double qx;
    double qy;
    double qz;
    double qw;

    CartesianQuaternion() :
        qx(0.0), qy(0.0), qz(0.0), qw(1.0)
    {
    }

    CartesianQuaternion(double qx_, double qy_, double qz_, double qw_) :
        qx(qx_), qy(qy_), qz(qz_), qw(qw_)
    {
    }

    
};

struct CartesianPose
{
    CartesianPosition position;
    CartesianOrientation orientation;
    CartesianQuaternion quat;

    CartesianPose()
    {
    }

    CartesianPose(const CartesianPosition &pos, const CartesianOrientation &ori, const CartesianQuaternion &q) :
        position(pos), orientation(ori), quat(q)
    {
    }

    CartesianPose(double x, double y, double z, double A, double B, double C, double qx, double qy, double qz, double qw) :
        position(x, y, z), orientation(A, B, C), quat(qx, qy, qz, qw)
    {
    }

    CartesianPose(double x, double y, double z, double A, double B, double C)
    {
        Eigen::Matrix3d matrix;
        matrix =
            Eigen::AngleAxisd(A, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(B, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(C, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quaternion(matrix);


        position = CartesianPosition(x, y, z);
        orientation = CartesianOrientation(A, B, C);
        quat = CartesianQuaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
    }
};

struct TimestampePose
{
    CartesianPose pose;
    uint64_t timestamp_ms;
    uint64_t button_mask;
};



/**
 * @brief 操作模式
 */
enum NRC_OperationMode
{
	NRC_TEACH_ = 0,     ///< 示教模式
	NRC_REMOTE_ = 1,     ///< 远程模式
	NRC_RUN_ = 2     ///< 运行模式
};

/**
 * @brief 机器人坐标系
 */
enum NRC_COORD
{
	NRC_ACS = 0,     ///< 关节坐标
	NRC_MCS = 1,     ///< 世界坐标
	NRC_PCS = 2,     ///< 工具坐标
	NRC_UCS = 3      ///< 用户坐标
};

enum NRC_ROBOT
{
    NRC_FIRST_ROBOT=1,
    NRC_ROLLER_ROBOT=2,
    NRC_THIRD_ROBOT=3,
    NRC_FOUR_ROBOT=4,
    NRC_ROBOT_END
};