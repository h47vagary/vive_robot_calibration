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

private:
    static void recv_callback_wrapper(int messageID, const char* message);

    static NrcComm* instance_;  // 用于回调静态绑定

    SOCKETFD socket_fd_;
    std::atomic<bool> connected_;
    MessageCallback callback_;
    mutable std::mutex mutex_;
};