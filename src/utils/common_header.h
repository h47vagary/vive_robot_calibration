#pragma once

#include <string>
#include <vector>
#include <map>

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

struct CartesianPose
{
    CartesianPosition position;
    CartesianOrientation orientation;

    CartesianPose()
    {
    }

    CartesianPose(const CartesianPosition &pos, const CartesianOrientation &ori) :
        position(pos), orientation(ori)
    {
    }

    CartesianPose(double x, double y, double z, double A, double B, double C) :
        position(x, y, z), orientation(A, B, C)
    {
    }
};

struct TimestampePose
{
    CartesianPose pose;
    uint64_t timestamp_us;
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