#pragma once

#include <thread>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

#include "common_header.h"
#include "utils.h"

struct VivePose
{
    float x, y, z;              // mm
    float qx, qy, qz, qw;
    uint64_t button_mask;
    bool valid;
};

class ViveTrackerReader
{
public:
    ViveTrackerReader();
    ~ViveTrackerReader();

    bool start();   // 启动读取线程
    void stop();    // 停止读取线程
    void pause();   // 暂停读取
    void resume();  // 恢复读取

    void enable_record(size_t max_size = 8000);                         // 开启轨迹缓存
    void disable_record();                                              // 停止轨迹缓存
    void set_loop_interval_ms(int interval_ms);                         // 设置读取间隔时间(ms)

    CartesianPose get_latest_pose();  // 获取最新点位
    std::vector<CartesianPose> get_recorded_poses();                    // 获取缓存的轨迹
    void clear_recorded_poses();                                        // 清空缓存的轨迹   
    bool save_record_poses_to_file(const std::string& filename, std::vector<CartesianPose> poses);        // 保存缓存的轨迹到文件(CSV格式)
    
    TimestampePose get_lastest_timepose();                              // 获取最新带时间戳点位
    std::vector<TimestampePose> get_recorded_timestamped_poses();       // 获取缓存的带时间戳轨迹
    void clear_recorded_timestamped_poses();
    bool save_record_timestamped_poses_to_file(const std::string& filename, std::vector<TimestampePose> poses);


private:
    void read_loop();
    void read_timestamped_loop();

    std::thread reader_thread_;
    std::thread reader_thread_timestampe_;
    std::atomic<bool> running_;
    std::atomic<bool> paused_;

    CartesianPose pose_buf_[2];                                 // 双缓冲区
    TimestampePose timestampe_pose_buf_[2];
    std::atomic<int> active_index_;                             // 当前可读缓冲区索引（0或1）

    std::atomic<bool> record_enabled_;                          // 是否启用轨迹缓存
    std::vector<CartesianPose> recorded_poses_;                 // 缓存的轨迹
    std::vector<TimestampePose> recorded_timestampe_poses_;     // 缓存的带时间戳的轨迹
    std::mutex record_mutex_;                                   // 轨迹缓存的互斥锁
    size_t max_record_size_;                                    // 最大缓存大小   
    size_t record_count_;                                       // 当前缓存的轨迹数量

    std::atomic<int> loop_interval_ms_;                         // 读取间隔时间(ms)
};
