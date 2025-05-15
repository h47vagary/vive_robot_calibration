#include "nrc_comm.h"

#include <iostream>

NrcComm* NrcComm::instance_ = nullptr;

NrcComm::NrcComm()
    : socket_fd_(-1),
      connected_(false)
{
    instance_ = this;
}

NrcComm::~NrcComm()
{
    disconnect();
    instance_ = nullptr;
}

bool NrcComm::connect(const std::string& ip, const std::string& port)
{
    socket_fd_ = connect_robot(ip, port);
    if (socket_fd_ < 0)
    {
        std::cerr << "连接失败\n";
        return false;
    }

    connected_ = true;

    // 注册接收回调
    recv_message(socket_fd_, recv_callback_wrapper);
    return true;
}

void NrcComm::disconnect()
{
    if (connected_)
    {
        disconnect_robot(socket_fd_);
        connected_ = false;
    }
}

bool NrcComm::is_connected() const
{
    return connected_;
}

bool NrcComm::send(int messageId, const std::string& message)
{
    if (!connected_)
        return false;

    return send_message(socket_fd_, messageId, message) == 0;
}

void NrcComm::set_callback(MessageCallback cb)
{
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = std::move(cb);
}

void NrcComm::recv_callback_wrapper(int messageID, const char* message)
{
    if (instance_)
    {
        std::lock_guard<std::mutex> lock(instance_->mutex_);
        if (instance_->callback_)
        {
            instance_->callback_(messageID, std::string(message));
        }
    }
}
