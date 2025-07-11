/**
 * @file message_handler.h
 * @brief 与控制器实际通讯组件
 * @version 0.1
 * @date 2025-05-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <QObject>

#include <atomic>
#include <memory>
#include <thread>
#include <string>

#include "comm_manager.h"
#include "common_header.h"

#define CONTROLLER_IP "192.168.1.13"
//#define CONTROLLER_IP "192.168.80.135"
#define CONTROLLER_PORT "6001"
#define CONTROLLER_SERIAL_ID 0x9206
#define CONTROLLER_RECEIVE_ID 0x9208

class MessageHandler : public QObject
{
    Q_OBJECT
public:
    explicit MessageHandler(std::shared_ptr<CommManager> comm, QObject* parent = nullptr);
    ~MessageHandler();

    void start();              // 启动线程并连接控制器
    void stop();               // 停止线程和通信

signals:
    void signal_flange2tcp_mark_point_received(int index, CartesianPose pose);
    void signal_compute_result_received(double result);
    void signal_get_linear_error_use_robot_pose(CartesianPose pose);
    

public slots:
    void slot_handler_start();                                      // 启动
    void slot_handler_stop();                                       // 停止
    void slot_handler_start_record();                               // 开始录制
    void slot_handler_end_record();                                 // 结束录制
    void slot_handler_start_playback();                             // 开始回放
    void slot_handler_end_playback();                               // 结束回放
    void slot_linear_error_acquire();                               // 获取线性误差（需要一次机器人姿态）

private:
    bool handle_pose_with_point(const std::string& msg, std::function<void(int, const CartesianPose&)> emitter);
    void thread_loop();  // 后台线程处理
    void handle_message(int msg_id, const std::string& msg);
    int serial_id_send_;

    std::thread thread_;
    std::atomic_bool running_{false};
    std::shared_ptr<CommManager> comm_; // 控制器通信管理类

};
