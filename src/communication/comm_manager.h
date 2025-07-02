/**
 * @file comm_manager.h
 * @brief 调用 nrc_comm.h 功能组件，封装给开发者使用
 * @version 0.1
 * @date 2025-05-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once 

#include <functional>
#include <memory>
#include <string>
#include <vector>

class CommManager
{
public:
    using MessageCallback = std::function<void(int, const std::string&)>;

    CommManager();
    ~CommManager();

    
    bool nrc_init(const std::string& ip, const std::string& port);              // 初始化NRC通讯
    bool nrc_send_message(int message_id, const std::string& message);          // 往NRC发送消息
    void nrc_register_callback(MessageCallback callback);                       // 注册NRC回调
    int nrc_get_current_position_robot(int robot_num, int coord, std::vector<double>& pos);     // 获取机器人当前位置
    void nrc_shutdown();                                                        // 断开NRC连接

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};