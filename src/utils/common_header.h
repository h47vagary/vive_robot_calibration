#pragma once

#include <string>
#include <vector>
#include <map>

enum E_POSE_TYPE
{
    E_POSE_TYPE_ROBOT = 0,
    E_POSE_TYPE_VIVE,
};

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