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

        // 插值位置（线性）
        CartesianPosition pos;
        pos.x = p1.pose.position.x + ratio * (p2.pose.position.x - p1.pose.position.x);
        pos.y = p1.pose.position.y + ratio * (p2.pose.position.y - p1.pose.position.y);
        pos.z = p1.pose.position.z + ratio * (p2.pose.position.z - p1.pose.position.z);

        // 欧拉角转四元数
        Quaternion q1, q2, q_t;
        Utils::euler_ABC_to_quaternion(p1.pose.orientation.A, p1.pose.orientation.B, p1.pose.orientation.C, q1);
        Utils::euler_ABC_to_quaternion(p2.pose.orientation.A, p2.pose.orientation.B, p2.pose.orientation.C, q2);

        // 插值四元数
        Utils::slerp(q1, q2, ratio, q_t);

        // 四元数转欧拉角
        double A, B, C;
        Utils::quaternion_to_euler_ABC(q_t, A, B, C);

        output.push_back(TimestampePose{
            CartesianPose(pos, CartesianOrientation(A, B, C), CartesianQuaternion(q_t.x, q_t.y, q_t.z, q_t.w)),
            next_time,
            p1.button_mask
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

        // 使用已存储的四元数插值
        Quaternion q1(p1.pose.quat.qw, p1.pose.quat.qx, p1.pose.quat.qy, p1.pose.quat.qz);
        Quaternion q2(p2.pose.quat.qw, p2.pose.quat.qx, p2.pose.quat.qy, p2.pose.quat.qz);
        Quaternion q_t;
        Utils::slerp(q1, q2, ratio, q_t);
        std::cout << "p1.pose.quat: " << p1.pose.quat.qw << ", " << p1.pose.quat.qx << ", " << p1.pose.quat.qy << ", " << p1.pose.quat.qz << std::endl;
        std::cout << "p2.pose.quat: " << p2.pose.quat.qw << ", " << p2.pose.quat.qx << ", " << p2.pose.quat.qy << ", " << p2.pose.quat.qz << std::endl;
        std::cout << "q_t: " << q_t.w << ", " << q_t.x << ", " << q_t.y << ", " << q_t.z << std::endl;

        // 四元数转欧拉角（用于可视化/兼容）
        double A, B, C;
        Utils::quaternion_to_euler_ABC(q_t, A, B, C);
        std::cout << "Interpolated Euler angles: A: " << A << ", B: " << B << ", C: " << C << std::endl;

        output.push_back(TimestampePose{
            CartesianPose(pos, CartesianOrientation(A, B, C), CartesianQuaternion(q_t.x, q_t.y, q_t.z, q_t.w)),
            next_time,
            p1.button_mask
        });
    }

    return true;
}
