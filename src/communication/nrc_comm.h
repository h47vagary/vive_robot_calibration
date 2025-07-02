/**
 * @file nrc_comm.h
 * @brief 封装nrc库的通讯功能组件
 * @version 0.1
 * @date 2025-05-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <string>
#include <functional>
#include <mutex>
#include <atomic>
#include "nrc_interface.h"

class NrcComm
{
public:
    using MessageCallback = std::function<void(int messageId, const std::string& message)>;

    NrcComm();
    ~NrcComm();

    bool connect(const std::string& ip, const std::string& port);
    void disconnect();
    bool is_connected() const;

    bool send(int messageId, const std::string& message);

    void set_callback(MessageCallback cb);
    int nrc_get_current_position_robot(int robot_num, int corrd, std::vector<double>& pos);

private:
    static void recv_callback_wrapper(int messageID, const char* message);

    static NrcComm* instance_;  // 用于回调静态绑定

    SOCKETFD socket_fd_;
    std::atomic<bool> connected_;
    MessageCallback callback_;
    mutable std::mutex mutex_;
};