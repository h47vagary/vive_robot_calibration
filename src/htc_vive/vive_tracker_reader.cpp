#include "vive_tracker_reader.h"
#include "vive_wrapper.h"

#include <iostream>
#include <fstream>
#include <chrono>

ViveTrackerReader::ViveTrackerReader()
    : running_(false), paused_(false), active_index_(0), 
    record_enabled_(false), max_record_size_(5000), loop_interval_ms_(9)
{

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

void ViveTrackerReader::enable_record(size_t max_size)
{
    std::lock_guard<std::mutex> lock(record_mutex_);
    recorded_poses_.clear();
    max_record_size_ = max_size;
    record_enabled_ = true;
}

void ViveTrackerReader::disable_record()
{
    std::lock_guard<std::mutex> lock(record_mutex_);
    recorded_poses_.clear();
    record_enabled_ = false;
}

std::vector<CartesianPose> ViveTrackerReader::get_recorded_poses()
{
    std::lock_guard<std::mutex> lock(record_mutex_);
    return recorded_poses_;
}

void ViveTrackerReader::clear_recorded_poses()
{
    std::lock_guard<std::mutex> lock(record_mutex_);
    recorded_poses_.clear();
}

bool ViveTrackerReader::save_record_poses_to_file(const std::string &filename)
{
    std::lock_guard<std::mutex> lock(record_mutex_);

    std::ofstream file(filename);
    if (!file.is_open()) 
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    file << "x,y,z,A,B,C\n";
    for (const auto& pose : recorded_poses_) 
    {
        file << pose.position.x << ","
             << pose.position.y << ","
             << pose.position.z << ","
             << pose.orientation.A << ","
             << pose.orientation.B << ","
             << pose.orientation.C << "\n";
    }

    return true;
}

void ViveTrackerReader::set_loop_interval_ms(int interval_ms)
{
    if (interval_ms > 0) 
    {
        loop_interval_ms_.store(interval_ms, std::memory_order_relaxed);
    } 
    else 
    {
        std::cerr << "Invalid loop interval: " << interval_ms << " ms" << std::endl;
    }
}

void ViveTrackerReader::read_loop()
{
    int write_index = 0;

    while (running_) {
        if (!paused_) {
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

            // 切换可读索引
            active_index_.store(write_index, std::memory_order_release);
            write_index = 1 - write_index;

            if (ok && record_enabled_)
            {
                std::lock_guard<std::mutex> lock(record_mutex_);
                if (recorded_poses_.size() >= max_record_size_)
                {
                    record_enabled_ = false;
                    std::cerr << "Recording buffer is full. Stopping recording." << std::endl;
                }
                else
                {
                    recorded_poses_.push_back(pose);
                }
                
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_interval_ms_));
    }
}
