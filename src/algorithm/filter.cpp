#include "filter.h"
#include "utils.h"

MovingAverageFilter::MovingAverageFilter(size_t window_size)
    : window_size_(window_size) {}

void MovingAverageFilter::set_filter_param(size_t window_size)
{
    window_size_ = window_size;
}

double MovingAverageFilter::update(double new_value)
{
    if (data_.size() >= window_size_)
    {
        data_.pop_front(); // 移除最旧的数据点
    }
    data_.push_back(new_value); // 添加新的数据点

    return get_average(); // 返回当前的移动平均值
}

double MovingAverageFilter::get_average() const
{
    if (data_.empty())
    {
        return 0.0; // 如果队列为空，返回0或抛出异常
    }

    double sum = std::accumulate(data_.begin(), data_.end(), 0.0); // 计算所有元素的总和
    return sum / data_.size(); // 返回平均值
}

void MovingAverageFilter::filter_position(std::vector<CartesianPose>& trajectory)
{
    if (trajectory.empty()) return;

    MovingAverageFilter fx(window_size_), fy(window_size_), fz(window_size_);
    for (auto& pose : trajectory)
    {
        pose.position.x = fx.update(pose.position.x);
        pose.position.y = fy.update(pose.position.y);
        pose.position.z = fz.update(pose.position.z);
    }
}

void MovingAverageFilter::filter_orientation(std::vector<CartesianPose>& trajectory)
{
    if (trajectory.empty()) return;

    // 四元数滑动窗口滤波器
    MovingAverageFilter fx_q(window_size_);
    MovingAverageFilter fy_q(window_size_);
    MovingAverageFilter fz_q(window_size_);
    MovingAverageFilter fw_q(window_size_);

    Quaternion q_prev, q_cur, q_filtered;

    // 处理第一个点
    Utils::euler_ABC_to_quaternion(trajectory[0].orientation.A,
                                   trajectory[0].orientation.B,
                                   trajectory[0].orientation.C,
                                   q_cur);

    // 填入初始值
    q_filtered.x = fx_q.update(q_cur.x);
    q_filtered.y = fy_q.update(q_cur.y);
    q_filtered.z = fz_q.update(q_cur.z);
    q_filtered.w = fw_q.update(q_cur.w);
    q_filtered.normalize();

    Utils::quaternion_to_euler_ABC(q_filtered,
                                   trajectory[0].orientation.A,
                                   trajectory[0].orientation.B,
                                   trajectory[0].orientation.C);

    q_prev = q_cur;

    for (size_t i = 1; i < trajectory.size(); ++i)
    {
        Utils::euler_ABC_to_quaternion(trajectory[i].orientation.A,
                                       trajectory[i].orientation.B,
                                       trajectory[i].orientation.C,
                                       q_cur);

        // 保证方向一致，避免跳变
        if (q_prev.dot(q_cur) < 0)
        {
            q_cur = -q_cur;
        }

        q_filtered.x = fx_q.update(q_cur.x);
        q_filtered.y = fy_q.update(q_cur.y);
        q_filtered.z = fz_q.update(q_cur.z);
        q_filtered.w = fw_q.update(q_cur.w);
        q_filtered.normalize();

        Utils::quaternion_to_euler_ABC(q_filtered,
                                       trajectory[i].orientation.A,
                                       trajectory[i].orientation.B,
                                       trajectory[i].orientation.C);

        q_prev = q_cur;
    }
}

void MovingAverageFilter::filter_xyzabc(std::vector<double>& x,
                                        std::vector<double>& y,
                                        std::vector<double>& z,
                                        std::vector<double>& a,
                                        std::vector<double>& b,
                                        std::vector<double>& c)
{
    size_t n = std::min({x.size(), y.size(), z.size(), a.size(), b.size(), c.size()});
    if (n == 0) return;

    // 平滑位置
    MovingAverageFilter fx_pos(window_size_), fy_pos(window_size_), fz_pos(window_size_);
    for (size_t i = 0; i < n; ++i)
    {
        x[i] = fx_pos.update(x[i]);
        y[i] = fy_pos.update(y[i]);
        z[i] = fz_pos.update(z[i]);
    }

    // 平滑姿态（四元数）
    Quaternion q_prev, q_cur, q_filtered;
    Utils::euler_ABC_to_quaternion(a[0], b[0], c[0], q_prev);

    MovingAverageFilter fx_q(window_size_), fy_q(window_size_), fz_q(window_size_), fw_q(window_size_);
    for (size_t i = 1; i < n; ++i)
    {
        Utils::euler_ABC_to_quaternion(a[i], b[i], c[i], q_cur);

        if (q_prev.dot(q_cur) < 0)
            q_cur = -q_cur;

        q_filtered.x = fx_q.update(q_cur.x);
        q_filtered.y = fy_q.update(q_cur.y);
        q_filtered.z = fz_q.update(q_cur.z);
        q_filtered.w = fw_q.update(q_cur.w);
        q_filtered.normalize();

        Utils::quaternion_to_euler_ABC(q_filtered, a[i], b[i], c[i]);
        q_prev = q_cur;
    }
}