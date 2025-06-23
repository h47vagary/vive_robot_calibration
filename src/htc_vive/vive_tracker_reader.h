#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <memory>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
#endif

#include "common_header.h"
#include "utils.h"

class TimerPrecisionGuard
{
public:
#ifdef _WIN32
    TimerPrecisionGuard(unsigned int ms = 1) { timeBeginPeriod(ms); }   // 提高系统定时器精度到 1ms
    ~TimerPrecisionGuard() { timeEndPeriod(1); }                        // 恢复系统定时器粒度
#else
    TimerPrecisionGuard(unsigned int ms = 1) {}
    ~TimerPrecisionGuard() {}
#endif
};

class ViveTrackerReader
{
public:
    ViveTrackerReader();
    ~ViveTrackerReader();

    bool start();                                                                                   // 启动读取线程
    void stop();                                                                                    // 停止读取线程
    void pause();                                                                                   // 暂停读取
    void resume();                                                                                  // 恢复读取

    void enable_record(size_t max_size = 8000);                                                     // 开启轨迹缓存
    void disable_record();                                                                          // 停止轨迹缓存
    void set_loop_interval_ms(int interval_ms);                                                     // 设置读取间隔时间(ms) 
    
    CartesianPose get_latest_pose();                                                                // 获取最新点位
    std::vector<TimestampePose> get_recorded_poses();                                               // 获取缓存的带时间戳轨迹
    void clear_recorded_poses();                                                                    // 清空缓存的轨迹 
    bool save_record_poses_to_file(const std::string& filename, std::vector<TimestampePose> poses); // 保存缓存的轨迹到文件(CSV格式)

    double get_record_duration_seconds() const;


private:
    void read_loop();

    std::thread reader_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> paused_;
   
    TimestampePose pose_buf_[2];                                // 双缓冲区
    std::atomic<int> active_index_;                             // 当前可读缓冲区索引（0或1）

    std::atomic<bool> record_enabled_;                          // 是否启用轨迹缓存
    std::vector<TimestampePose> recorded_poses_;                // 缓存的带时间戳的轨迹
    size_t max_record_size_;                                    // 最大缓存大小   
    size_t record_count_;                                       // 当前缓存的轨迹数量

    std::atomic<int> loop_interval_ms_;                         // 读取间隔时间(ms)
    uint64_t record_start_timestamp_us_ = 0;
    uint64_t record_duration_us_ = 0;       

#ifdef _WIN32
    std::unique_ptr<TimerPrecisionGuard> timer_guard_;
#endif
};