#pragma once 

#include <string>
#include <functional>
#include <memory>

class CommManager
{
public:
    using MessageCallback = std::function<void(int, const std::string&)>;

    CommManager();
    ~CommManager();

    
    bool nrc_init(const std::string& ip, const std::string& port);              // 初始化NRC通讯
    bool nrc_send_message(int message_id, const std::string& message);          // 往NRC发送消息
    void nrc_register_callback(MessageCallback callback);                       // 注册NRC回调
    void nrc_shutdown();                                                        // 断开NRC连接

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};