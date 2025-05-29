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
