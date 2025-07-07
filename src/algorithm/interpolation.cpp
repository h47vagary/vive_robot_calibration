#include "interpolation.h"
#include "utils.h"

LinearPoseInterpolator::LinearPoseInterpolator(uint64_t interval_us)
    : interval_us_(interval_us)
{
}

void LinearPoseInterpolator::set_interval_us(uint64_t interval_us)
{
    interval_us = interval_us_;
}

bool LinearPoseInterpolator::interpolate_with_eular(const std::vector<TimestampePose> &input, std::vector<TimestampePose> &output)
{
    if (input.size() < 2) return false;

    output.clear();
    output.push_back(input[0]);
    size_t curr_idx = 0;
    uint64_t next_time = input[0].timestamp_ms;

    while (curr_idx < input.size() - 1)
    {
        const auto &p1 = input[curr_idx];
        const auto &p2 = input[curr_idx + 1];

        if (next_time + interval_us_ > p2.timestamp_ms)
        {
            ++curr_idx;
            continue;
        }

        next_time += interval_us_;
        double ratio = static_cast<double>(next_time - p1.timestamp_ms) / (p2.timestamp_ms - p1.timestamp_ms);

        CartesianPosition pos;
        pos.x = p1.pose.position.x + ratio * (p2.pose.position.x - p1.pose.position.x);
        pos.y = p1.pose.position.y + ratio * (p2.pose.position.y - p1.pose.position.y);
        pos.z = p1.pose.position.z + ratio * (p2.pose.position.z - p1.pose.position.z);

        CartesianOrientation ori;
        ori.A = p1.pose.orientation.A + ratio * (p2.pose.orientation.A - p1.pose.orientation.A);
        ori.B = p1.pose.orientation.B + ratio * (p2.pose.orientation.B - p1.pose.orientation.B);
        ori.C = p1.pose.orientation.C + ratio * (p2.pose.orientation.C - p1.pose.orientation.C);

        output.push_back(TimestampePose{
            CartesianPose(pos, ori), 
            next_time,
            p1.button_mask      // 沿用当前段起点的 button_mask
        });
    }
    return true;
}

bool LinearPoseInterpolator::interpolate_with_quaternion(const std::vector<TimestampePose> &input, std::vector<TimestampePose> &output)
{
    if (input.size() < 2) return false;

    output.clear();
    output.push_back(input[0]);
    size_t curr_idx = 0;
    uint64_t next_time = input[0].timestamp_ms;

    while (curr_idx < input.size() - 1)
    {
        const auto &p1 = input[curr_idx];
        const auto &p2 = input[curr_idx + 1];

        if (next_time + interval_us_ > p2.timestamp_ms)
        {
            ++curr_idx;
            continue;
        }

        next_time += interval_us_;
        double ratio = static_cast<double>(next_time - p1.timestamp_ms) / (p2.timestamp_ms - p1.timestamp_ms);

        // 插值位置（线性）
        CartesianPosition pos;
        pos.x = p1.pose.position.x + ratio * (p2.pose.position.x - p1.pose.position.x);
        pos.y = p1.pose.position.y + ratio * (p2.pose.position.y - p1.pose.position.y);
        pos.z = p1.pose.position.z + ratio * (p2.pose.position.z - p1.pose.position.z);

        // 欧拉角转四元数
        Quaternion q1, q2, q_t;
        Utils::euler_ABC_to_quaternion(p1.pose.orientation.A, p1.pose.orientation.B, p1.pose.orientation.C, q1);
        Utils::euler_ABC_to_quaternion(p2.pose.orientation.A, p2.pose.orientation.B, p2.pose.orientation.C, q2);

        // slerp 插值
        Utils::slerp(q1, q2, ratio, q_t);

        // 四元数转欧拉角
        CartesianOrientation ori;
        Utils::quaternion_to_euler_ABC(q_t, ori.A, ori.B, ori.C);

        // 输出插值后的姿态
        output.push_back(TimestampePose{
            CartesianPose(pos, ori), 
            next_time,
            p1.button_mask      // 沿用当前段起点的 button_mask
        });
    }

    return true;
}

