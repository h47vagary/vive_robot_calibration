#pragma once

#include <thread>
#include <atomic>
#include <cstdint>

struct VivePose
{
    float x, y, z;
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

    VivePose get_latest_pose();  // 获取最新点位

private:
    void read_loop();

    std::thread reader_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> paused_;

    VivePose pose_buf_[2];            // 双缓冲区
    std::atomic<int> active_index_;   // 当前可读缓冲区索引（0或1）
};
