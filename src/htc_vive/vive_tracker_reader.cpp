#include "vive_tracker_reader.h"
#include "vive_wrapper.h"
#include "interpolation.h"

#include <iostream>
#include <fstream>
#include <chrono>

ViveTrackerReader::ViveTrackerReader()
    : running_(false), paused_(false), active_index_(0), 
    record_enabled_(false), max_record_size_(8000), loop_interval_ms_(4)
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
    const TimestampePose& cartesian_pose = pose_buf_[index];
    return cartesian_pose.pose;
}

std::vector<TimestampePose> ViveTrackerReader::get_recorded_poses()
{
    return std::vector<TimestampePose>(recorded_poses_.begin(), recorded_poses_.begin() + record_count_);
}

void ViveTrackerReader::clear_recorded_poses()
{
    recorded_poses_.clear();
}

bool ViveTrackerReader::save_record_poses_to_file(const std::string &filename, std::vector<TimestampePose> poses)
{
    std::ofstream file(filename);
    if (!file.is_open()) 
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    file << "x,y,z,A,B,C,T,delta_ms\n";

    uint64_t last_timestamp = 0;

    for (size_t i = 0; i < poses.size(); ++i)
    {
        const auto& timestampe_pose = poses[i];

        uint64_t curr_timestamp = timestampe_pose.timestamp_us;
        uint64_t delta_ms = 0;
        if (i > 0)
        {
            delta_ms = curr_timestamp - last_timestamp;
        }

        file << timestampe_pose.pose.position.x << ","
             << timestampe_pose.pose.position.y << ","
             << timestampe_pose.pose.position.z << ","
             << timestampe_pose.pose.orientation.A << ","
             << timestampe_pose.pose.orientation.B << ","
             << timestampe_pose.pose.orientation.C << ","
             << curr_timestamp << ","
             << delta_ms << "\n";

        last_timestamp = curr_timestamp;
    }

    return true;
}

double ViveTrackerReader::get_record_duration_seconds() const
{
    return record_duration_us_ / 1e3;
}

void ViveTrackerReader::enable_record(size_t max_size)
{
    max_record_size_ = max_size;
    recorded_poses_.clear();
    recorded_poses_.resize(max_record_size_);      // 预分配内存以提高性能
    recorded_poses_.clear();
    recorded_poses_.resize(max_record_size_);
    record_count_ = 0;
    record_start_timestamp_us_ = TimeDealUtils::get_timestamp();
    record_duration_us_ = 0;
    record_enabled_ = true;
}

void ViveTrackerReader::disable_record()
{
    record_enabled_ = false;
    uint64_t end_timestamp = TimeDealUtils::get_timestamp();
    record_duration_us_ = end_timestamp - record_start_timestamp_us_;
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

    while (running_) 
    {
        if (!paused_) 
        {
            uint64_t now_us = TimeDealUtils::get_timestamp();

            double x, y, z, A, B, C;
            uint64_t button_mask = 0;
            bool ok = vive_get_pose_abc(&x, &y, &z, &A, &B, &C, &button_mask);

            CartesianPose& pose = pose_buf_[write_index].pose;
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
                if (record_count_ >= max_record_size_)
                {
                    record_enabled_ = false;
                    std::cerr << "Recording buffer is full. Stopping recording." << std::endl;
                }
                else
                {
                    recorded_poses_[record_count_].pose = pose;
                    recorded_poses_[record_count_].timestamp_us = now_us;
                    record_count_++;
                }
                
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_interval_ms_));
    }
}
