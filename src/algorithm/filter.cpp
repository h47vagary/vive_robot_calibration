#include "filter.h"
#include "utils.h"

PoseFilter::PoseFilter(int window_size, int polynomial_order)
    : window_size_(window_size), polynomial_order_(polynomial_order)
{}

void PoseFilter::filter_position(std::vector<CartesianPose>& trajectory)
{
    if (trajectory.empty())
        return;

    const gram_sg::SavitzkyGolayFilterConfig conf(window_size_, window_size_, polynomial_order_, 0);
    gram_sg::EigenVectorFilter<double> fx(conf), fy(conf), fz(conf);

    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        fx.add(trajectory[i].position.x);
        fy.add(trajectory[i].position.y);
        fz.add(trajectory[i].position.z);

        if (fx.ready())
        {
            trajectory[i].position.x = fx.filter();
            trajectory[i].position.y = fy.filter();
            trajectory[i].position.z = fz.filter();
        }
    }
}

void PoseFilter::filter_orientation(std::vector<CartesianPose>& trajectory)
{
    if (trajectory.size() < 2)
        return;

    const gram_sg::SavitzkyGolayFilterConfig conf(window_size_, window_size_, polynomial_order_, 0);
    gram_sg::EigenVectorFilter<double> fx(conf), fy(conf), fz(conf), fw(conf);

    Quaternion q_prev, q_cur, q_filtered;

    Utils::euler_ABC_to_quaternion(trajectory[0].orientation.A, trajectory[0].orientation.B, trajectory[0].orientation.C, q_prev);

    for (size_t i = 1; i < trajectory.size(); ++i)
    {
        Utils::euler_ABC_to_quaternion(trajectory[i].orientation.A, trajectory[i].orientation.B, trajectory[i].orientation.C, q_cur);

        if (q_prev.dot(q_cur) < 0)
            q_cur = -q_cur;

        fx.add(q_cur.x);
        fy.add(q_cur.y);
        fz.add(q_cur.z);
        fw.add(q_cur.w);

        if (fx.ready())
        {
            q_filtered.x = fx.filter();
            q_filtered.y = fy.filter();
            q_filtered.z = fz.filter();
            q_filtered.w = fw.filter();
            q_filtered.normalize();
        }
        else
        {
            q_filtered = q_cur;
        }

        Utils::quaternion_to_euler_ABC(q_filtered, trajectory[i].orientation.A, trajectory[i].orientation.B, trajectory[i].orientation.C);
        q_prev = q_cur;
    }
}

void PoseFilter::filter_xyzabc(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &a, std::vector<double> &b, std::vector<double> &c)
{
    size_t n = std::min({x.size(), y.size(), z.size(), a.size(), b.size(), c.size()});
    if (n == 0)
        return;

    // --- 位置分量滤波 ---
    {
        gram_sg::SavitzkyGolayFilterConfig conf(window_size_, window_size_, polynomial_order_, 0);
        gram_sg::EigenVectorFilter<double> fx(conf), fy(conf), fz(conf);

        for (size_t i = 0; i < n; ++i)
        {
            fx.add(x[i]);
            fy.add(y[i]);
            fz.add(z[i]);

            if (fx.ready())
            {
                x[i] = fx.filter();
                y[i] = fy.filter();
                z[i] = fz.filter();
            }
        }
    }

    // --- 姿态分量滤波 ---
    {
        gram_sg::SavitzkyGolayFilterConfig conf(window_size_, window_size_, polynomial_order_, 0);
        gram_sg::EigenVectorFilter<double> fq_x(conf), fq_y(conf), fq_z(conf), fq_w(conf);

        Quaternion q_prev, q_cur, q_filtered;

        // 初始化前一个四元数
        Utils::euler_ABC_to_quaternion(a[0], b[0], c[0], q_prev);

        for (size_t i = 1; i < n; ++i)
        {
            Utils::euler_ABC_to_quaternion(a[i], b[i], c[i], q_cur);

            if (q_prev.dot(q_cur) < 0)
                q_cur = -q_cur;

            fq_x.add(q_cur.x);
            fq_y.add(q_cur.y);
            fq_z.add(q_cur.z);
            fq_w.add(q_cur.w);

            if (fq_x.ready())
            {
                q_filtered.x = fq_x.filter();
                q_filtered.y = fq_y.filter();
                q_filtered.z = fq_z.filter();
                q_filtered.w = fq_w.filter();
                q_filtered.normalize();
            }
            else
            {
                q_filtered = q_cur;
            }

            Utils::quaternion_to_euler_ABC(q_filtered, a[i], b[i], c[i]);
            q_prev = q_cur;
        }
    }
}
