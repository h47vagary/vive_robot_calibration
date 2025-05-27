#include "vive_tracker_reader.h"
#include "vive_wrapper.h"

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
    const CartesianPose& cartesian_pose = pose_buf_[index];
    return cartesian_pose;
}

void ViveTrackerReader::enable_record(size_t max_size)
{
    // std::lock_guard<std::mutex> lock(record_mutex_);
    max_record_size_ = max_size;
    recorded_poses_.clear();
    recorded_poses_.resize(max_record_size_);      // 预分配内存以提高性能
    record_count_ = 0;
    record_enabled_ = true;
}

void ViveTrackerReader::disable_record()
{
    // std::lock_guard<std::mutex> lock(record_mutex_);
    record_enabled_ = false;
}

std::vector<CartesianPose> ViveTrackerReader::get_recorded_poses()
{
    // std::lock_guard<std::mutex> lock(record_mutex_); 
    return std::vector<CartesianPose>(recorded_poses_.begin(), recorded_poses_.begin() + record_count_);
}

void ViveTrackerReader::clear_recorded_poses()
{
    // std::lock_guard<std::mutex> lock(record_mutex_);
    recorded_poses_.clear();
}

bool ViveTrackerReader::save_record_poses_to_file(const std::string &filename, std::vector<CartesianPose> poses)
{
    std::ofstream file(filename);
    if (!file.is_open()) 
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    file << "x,y,z,A,B,C\n";
    for (size_t i = 0; i < poses.size(); ++i)
    {
        const auto& pose = poses[i];
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

bool ViveTrackerReader::load_record_poses_from_file(const std::string &filename, std::vector<CartesianPose> &out_poses)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    out_poses.clear();

    std::string line;
    std::getline(file, line); // 跳过标题行

    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string token;
        CartesianPose pose;

        // 依次读取6个字段
        if (!std::getline(ss, token, ',')) return false;
        if (!parse_double(token, pose.position.x)) return false;

        if (!std::getline(ss, token, ',')) return false;
        if (!parse_double(token, pose.position.y)) return false;

        if (!std::getline(ss, token, ',')) return false;
        if (!parse_double(token, pose.position.z)) return false;

        if (!std::getline(ss, token, ',')) return false;
        if (!parse_double(token, pose.orientation.A)) return false;

        if (!std::getline(ss, token, ',')) return false;
        if (!parse_double(token, pose.orientation.B)) return false;

        if (!std::getline(ss, token, ',')) return false;
        if (!parse_double(token, pose.orientation.C)) return false;

        out_poses.push_back(pose);
    }

    return true;
}

bool ViveTrackerReader::parse_double(const std::string &str, double &value)
{
    char* endptr = nullptr;
    value = std::strtod(str.c_str(), &endptr);
    return endptr != str.c_str() && *endptr == '\0';
}

void ViveTrackerReader::read_loop()
{
    int write_index = 0;

    while (running_) 
    {
        if (!paused_) 
        {
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
                // TODO: 暂时无锁写入，后续如有稳定性问题，需改为无锁缓冲或双缓冲
                // std::lock_guard<std::mutex> lock(record_mutex_);
                if (record_count_ >= max_record_size_)
                {
                    record_enabled_ = false;
                    std::cerr << "Recording buffer is full. Stopping recording." << std::endl;
                }
                else
                {
                    recorded_poses_[record_count_] = pose;
                    record_count_++;
                }
                
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_interval_ms_));
    }
}
