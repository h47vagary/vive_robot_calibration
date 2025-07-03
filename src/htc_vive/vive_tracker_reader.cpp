#include "vive_tracker_reader.h"
#include "vive_wrapper.h"

#include <iostream>
#include <fstream>
#include <chrono>


ViveTrackerReader::ViveTrackerReader()
    : running_(false), paused_(false), active_index_(0), 
    record_enabled_(false), max_record_size_(8000), loop_interval_ms_(8),
    pose_fetch_mode_(PoseFetchMode::Blocking), record_count_(0)
{
}

ViveTrackerReader::~ViveTrackerReader()
{
    stop();
}

void ViveTrackerReader::set_pose_fetch_mode(PoseFetchMode mode)
{
    pose_fetch_mode_.store(mode, std::memory_order_relaxed);
}

bool ViveTrackerReader::start()
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

    reader_thread_ = std::thread([this]() {

    #ifdef _WIN32
        // if (!set_process_high_priority())
        // {
        //     std::cerr << "set process hig priority fail!" << std::endl;
        // }
        // if (!set_thread_high_priority())
        // {
        //     std::cerr << "thread high priority fail!" << std::endl;
        // }
        // if (!bind_thread_to_cpu(0))
        // {
        //     std::cerr << "bind thread to cpu fail!" << std::endl;
        // }
        // std::cout << "Thread priority is: " << GetThreadPriority(GetCurrentThread()) << std::endl;
    #endif

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
    bool ok = false;
    
    if (pose_fetch_mode_ == PoseFetchMode::Blocking)
    {
        ok = vive_get_pose_euler(&x, &y, &z, &A, &B, &C, &button_mask);
    }
    else
    {
        ok = vive_get_pose_euler_non_blocking(&x, &y, &z, &A, &B, &C, &button_mask);
    }
    

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

void ViveTrackerReader::stop()
{
    running_ = false;
    if (reader_thread_.joinable()) 
    {
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

    if (poses.empty())
        return true;

    file << "x,y,z,A,B,C,T,delta_ms\n";
    
    uint64_t last_timestamp = poses[0].timestamp_us;

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

void ViveTrackerReader::enable_record(size_t max_size)
{
    max_record_size_ = max_size;
    recorded_poses_.clear();
    recorded_poses_.resize(max_record_size_);

    record_count_ = 0;
    record_start_timestamp_us_ = TimeDealUtils::get_timestamp();
    record_enabled_ = true;
}

void ViveTrackerReader::disable_record()
{
    record_enabled_ = false;
    uint64_t end_timestamp = TimeDealUtils::get_timestamp();

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
#ifdef _WIN32
    timeBeginPeriod(1); // 设置最小定时器精度为1ms
#endif

    int write_index = 0;
    auto stats_start = std::chrono::steady_clock::now();
    uint64_t frame_count = 0;
    constexpr uint64_t STATS_INTERVAL = 100; // 每100帧输出一次统计信息
    auto next_loop_time = std::chrono::steady_clock::now();

    while (running_) 
    {
        next_loop_time += std::chrono::milliseconds(loop_interval_ms_);

        if (!paused_) 
        {
            auto loop_start = std::chrono::steady_clock::now();
            frame_count++;

            // 获取姿态数据
            double x = 0, y = 0, z = 0, A = 0, B = 0, C = 0;
            uint64_t button_mask = 0;
            bool ok = false;

            auto fetch_start = std::chrono::steady_clock::now();
            if (pose_fetch_mode_ == PoseFetchMode::Blocking) 
            {
                ok = vive_get_pose_euler(&x, &y, &z, &A, &B, &C, &button_mask);
            } 
            else
            {
                ok = vive_get_pose_euler_non_blocking(&x, &y, &z, &A, &B, &C, &button_mask);
            }
            auto fetch_end = std::chrono::steady_clock::now();
            auto fetch_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(fetch_end - fetch_start).count();
            // std::cout << "[POSE FETCH] Duration: " << fetch_duration_us << " us"
            // << " | Mode: " << (pose_fetch_mode_ == PoseFetchMode::Blocking ? "Blocking" : "NonBlocking")
            // << " | OK: " << ok << " | x: " << x << " | y: " << y << " | z: " << z << " | A: " << A << " | B: " << B << " | C: " << C << std::endl;


            // 更新姿态缓冲区
            CartesianPose& pose = pose_buf_[write_index].pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.A = A;
            pose.orientation.B = B;
            pose.orientation.C = C;

            active_index_.store(write_index, std::memory_order_release);
            write_index = 1 - write_index;

            // 记录数据
            if (ok && record_enabled_)
            {
                if (record_count_ >= max_record_size_)
                {
                    record_enabled_ = false;
                    std::cerr << "Recording buffer is full (" << max_record_size_ 
                              << " poses). Stopping recording." << std::endl;
                }
                else
                {
                    recorded_poses_[record_count_].pose = pose;
                    recorded_poses_[record_count_].timestamp_us = 
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            loop_start.time_since_epoch()).count();
                    record_count_++;
                }
            }

            // 定期输出统计信息
            #if 0
            if (frame_count % STATS_INTERVAL == 0)
            {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - stats_start).count();
                
                // 防止除以零和异常大的时间差
                if (elapsed < 1000) elapsed = 1000; // 最小1ms防止计算异常
                
                double fps = STATS_INTERVAL / (elapsed / 1e6);
                int64_t jitter = std::chrono::duration_cast<std::chrono::microseconds>(
                    now - next_loop_time).count();
                
                std::cout << "Tracker FPS: " << fps 
                          << " | Mode: " << (pose_fetch_mode_ == PoseFetchMode::Blocking ? "Blocking" : "NonBlocking")
                          << " | Buffer: " << record_count_ << "/" << max_record_size_
                          << " | Loop jitter: " << jitter << "us"
                          << std::endl;
                 
                stats_start = now;
            }
            #endif
        }
        
        auto now = std::chrono::steady_clock::now();
        if (now < next_loop_time) {
            std::this_thread::sleep_until(next_loop_time);
        } else {
            // std::cerr << "Frame overrun by " 
            //           << std::chrono::duration_cast<std::chrono::microseconds>(
            //               now - next_loop_time).count() 
            //           << "us" << std::endl;
        }
    }
#ifdef _WIN32
    timeEndPeriod(1);
#endif
}
