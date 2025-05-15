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
};

struct CartesianPose
{
    CartesianPosition position;
    CartesianOrientation orientation;
};