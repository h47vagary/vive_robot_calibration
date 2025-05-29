#pragma once
/**
 * @file filter.h
 * @brief 滤波算法器
 * @version 0.1
 * @date 2025-05-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <vector>
#include "common_header.h"
#include "gram_savitzky_golay.h"
#include "spatial_filters.h"


class PoseFilter
{
public:
    PoseFilter(int window_size, int polynomial_order = 2);

    void filter_position(std::vector<CartesianPose>& trajectory);
    void filter_orientation(std::vector<CartesianPose>& trajectory);

private:
    int window_size_;
    int polynomial_order_;
};