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
#include <deque>
#include <numeric>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <iostream>
#include "common_header.h"
#include "gram_savitzky_golay.h"
#include "spatial_filters.h"


enum FilterType
{
    SavitzkyGolay,
    MovingAverage
};

class MovingAverageFilter
{
public:
    // 构造函数初始化窗口大小
    explicit MovingAverageFilter(size_t window_size) : window_size_(window_size) {}

    // 添加新的数据点并返回当前的移动平均值
    double update(double new_value)
    {
        if (data_.size() == window_size_)
        {
            // 如果队列已满，则移除最早的数据点
            data_.pop_front();
        }
        // 添加新的数据点到队列
        data_.push_back(new_value);
        // 计算并返回移动平均值
        return get_average();
    }

private:
    // 获取当前的移动平均值
    double get_average() const
    {
        if (data_.empty())
        {
            return 0.0; // 或者可以抛出异常，根据需要处理
        }
        // 使用 std::accumulate 来计算所有元素的总和
        double sum = std::accumulate(data_.begin(), data_.end(), 0.0);
        // 返回平均值
        return sum / data_.size();
    }

private:
    size_t window_size_; // 窗口大小
    std::deque<double> data_; // 用于存储数据点的双端队列
};


class PoseFilter
{
public:
    PoseFilter(int window_size, int polynomial_order = 2, FilterType filter_type = FilterType::SavitzkyGolay);

    void set_filter_param(int window_size, int polynomial_order, FilterType filter_type);
    void filter_position(std::vector<CartesianPose>& trajectory);
    void filter_orientation(std::vector<CartesianPose>& trajectory);
    void filter_xyzabc(std::vector<double>& x,
                       std::vector<double>& y,
                       std::vector<double>& z,
                       std::vector<double>& a,
                       std::vector<double>& b,
                       std::vector<double>& c);

private:
    int window_size_;
    int polynomial_order_;
    FilterType filter_type_;
};