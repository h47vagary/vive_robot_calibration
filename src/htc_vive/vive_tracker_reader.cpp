#include "vive_tracker_reader.h"
#include "vive_wrapper.h"

#include <chrono>

ViveTrackerReader::ViveTrackerReader()
    : running_(false), paused_(false), active_index_(0)
{
    // pose_buf_[0] = {0, 0, 0, 0, 0, 0, 1, 0, false};
    // pose_buf_[1] = pose_buf_[0];
}

ViveTrackerReader::~ViveTrackerReader()
{
    stop();
}

bool ViveTrackerReader::start()
{
    if (!vive_initialize())
        return false;

    if (!vive_find_tracker()) {
        vive_shutdown();
        return false;
    }

    running_ = true;
    paused_ = false;
    reader_thread_ = std::thread(&ViveTrackerReader::read_loop, this);
    return true;
}

void ViveTrackerReader::stop()
{
    running_ = false;
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    vive_shutdown();
}

void ViveTrackerReader::pause()
{
    paused_ = true;
}

void ViveTrackerReader::resume()
{
    paused_ = false;
}

CartesianPose ViveTrackerReader::get_latest_pose()
{
    int index = active_index_.load(std::memory_order_acquire);
    // const VivePose& pose = pose_buf_[index];

    // CartesianPose cartesian_pose;
    // cartesian_pose.position.x = pose.x;
    // cartesian_pose.position.y = pose.y;
    // cartesian_pose.position.z = pose.z;

    // // 四元数转欧拉角 (ABC)
    // Quaternion quat(pose.qw, pose.qx, pose.qy, pose.qz);
    // quat = quat.normalize();  // 确保单位四元数
    // Utils::quaternion_to_euler_ABC(quat, 
    //                                 cartesian_pose.orientation.A,
    //                                 cartesian_pose.orientation.B,
    //                                 cartesian_pose.orientation.C);

    // std::cout << "x:" << cartesian_pose.position.x << " y:" << cartesian_pose.position.y << " z:" << cartesian_pose.position.z << 
    //             " A:" << cartesian_pose.orientation.A << " B:" << cartesian_pose.orientation.B << " C:" << cartesian_pose.orientation.C << std::endl;
    

    const CartesianPose& cartesian_pose = pose_buf_[index];
    return cartesian_pose;
}

void ViveTrackerReader::read_loop()
{
    int write_index = 0;

    while (running_) {
        if (!paused_) {
            // float x, y, z, qx, qy, qz, qw;
            // uint64_t button_mask = 0;
            // bool ok = vive_get_pose(&x, &y, &z, &qx, &qy, &qz, &qw, &button_mask);

            double x, y, z, A, B, C;
            uint64_t button_mask = 0;
            bool ok = vive_get_pose_abc(&x, &y, &z, &A, &B, &C, &button_mask);

            CartesianPose& pose = pose_buf_[write_index];
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.A = A;
            pose.orientation.B = B;
            pose.orientation.C = C;

            // VivePose& pose = pose_buf_[write_index];
            // pose.x = x;
            // pose.y = y;
            // pose.z = z;
            // pose.qx = qx;
            // pose.qy = qy;
            // pose.qz = qz;
            // pose.qw = qw;
            // pose.button_mask = button_mask;
            // pose.valid = ok;

            // std::cout << "x:" << pose.x << " y:" << pose.y << " z:" << pose.z << " qx:" << pose.qx << " qy:" << pose.qy << " qz:" << 
            //                     pose.qz << " qw:" << pose.qw << std::endl;

            // 切换可读索引
            active_index_.store(write_index, std::memory_order_release);
            write_index = 1 - write_index;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(9));
    }
}
