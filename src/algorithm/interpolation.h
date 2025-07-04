/**
 * @file interpolation.h
 * @brief 插值算法器
 * @version 0.1
 * @date 2025-05-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <vector>
#include <cstdint>
#include "common_header.h"

// todo: 插补器应该只负责自己的插补内容，不应该对 TimestampePose 的字段增加修改进行适配。后续其他字段通过策略交给调用者：

/**
 * @brief 以固定时间间隔，递推进行线式插补。等时间间隔生成新数据点
 * 
 */
class LinearPoseInterpolator
{
public:
    explicit LinearPoseInterpolator(uint64_t interval_us);
    void set_interval_us(uint64_t interval_us);
    bool interpolate_with_eular(const std::vector<TimestampePose> &input, std::vector<TimestampePose> &output);
    bool interpolate_with_quaternion(const std::vector<TimestampePose> &input, std::vector<TimestampePose> &output);


private:
    uint64_t interval_us_;
};
