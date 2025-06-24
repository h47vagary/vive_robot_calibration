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

#ifdef _WIN32
    //timer_guard_ = std::make_unique<TimerPrecisionGuard>();
#endif

    if (!vive_find_tracker()) 
    {
#ifdef _WIN32
        //timer_guard_.reset();
#endif
        vive_shutdown();
        return false;
    }

    running_ = true;
    paused_ = false;

    reader_thread_ = std::thread([this]() {

    //#ifdef _WIN32
    #if     0
        if (!set_process_high_priority())
        {
            std::cerr << "set process hig priority fail!" << std::endl;
        }
        if (!set_thread_high_priority())
        {
            std::cerr << "thread high priority fail!" << std::endl;
        }
        if (!bind_thread_to_cpu(2))
        {
            std::cerr << "bind thread to cpu fail!" << std::endl;
        }
    #endif

        std::cout << "Thread priority is: " << GetThreadPriority(GetCurrentThread()) << std::endl;

        this->read_loop();

    });

    return true;
}


void ViveTrackerReader::on_timer_tick()
{
    if (!running_ || paused_) return;

    uint64_t now_us = TimeDealUtils::get_timestamp();

    double x, y, z, A, B, C;
    uint64_t button_mask = 0;
    bool ok = vive_get_pose_abc(&x, &y, &z, &A, &B, &C, &button_mask);

    static int write_index = 0;
    CartesianPose& pose = pose_buf_[write_index].pose;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.A = A;
    pose.orientation.B = B;
    pose.orientation.C = C;

    active_index_.store(write_index, std::memory_order_release);
    write_index = 1 - write_index;

    if (ok && record_enabled_)
    {
        if (record_count_ >= max_record_size_)
        {
            record_enabled_ = false;
            std::cerr << "Recording buffer full, stopping." << std::endl;
        }
        else
        {
            recorded_poses_[record_count_].pose = pose;
            recorded_poses_[record_count_].timestamp_us = now_us;
            record_count_++;
        }
    }
}


void CALLBACK ViveTrackerReader::timer_callback(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{
    ViveTrackerReader* self = reinterpret_cast<ViveTrackerReader*>(dwUser);
    if (self) self->on_timer_tick();
}

bool ViveTrackerReader::start_for_timer()
{
    if (!vive_initialize())
        return false;

    if (!vive_find_tracker())
    {
        vive_shutdown();
        return false;
    }

    running_ = true;
    paused_ = false;

    timeBeginPeriod(1);     // 设置全局定时器精度

    timer_id_ = timeSetEvent (
        loop_interval_ms_,      // 周期(ms)
        0,                      // 精度
        timer_callback,         // 回调函数
        (DWORD_PTR)this,        // 传入(this 指针)
        TIME_PERIODIC           // 周期模式
    );

    if (timer_id_ == 0)
    {
        std::cerr << "Failed to create high-precision timer." << std::endl;
        timeEndPeriod(1);
        return false;
    }

    return true;
}

void ViveTrackerReader::stop()
{
    running_ = false;
    if (reader_thread_.joinable()) 
    {
        reader_thread_.join();
    }
    vive_shutdown();

#ifdef _WIN32
    //timer_guard_.reset();
#endif

}

void ViveTrackerReader::stop_for_timer()
{
    running_ = false;

    if (timer_id_ != 0)
    {
        timeKillEvent(timer_id_);
        timer_id_ = 0;
        timeEndPeriod(1);
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

    // // 内存锁定防止分页
    // if (!lock_memory_region(recorded_poses_.data(), recorded_poses_.size() * sizeof(TimestampePose)))
    // {
    //     std::cerr << "Failed to lock memory for recorded_poses_" << std::endl;
    // }

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

    //unlock_memory_region(recorded_poses_.data(), recorded_poses_.size() * sizeof(TimestampePose));
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
