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
#include <thread>
#include <atomic>
#include <string>
#include "comm_manager.h"
#include "common_header.h"

class MessageHandler : public QObject
{
    Q_OBJECT

public:
    explicit MessageHandler(QObject* parent = nullptr);
    ~MessageHandler();

    void start();              // 启动线程并连接控制器
    void stop();               // 停止线程和通信

signals:
    void signal_message_received(QString msg);
    void signal_mark_point_received(E_POSE_TYPE type, int index, CartesianPose pose);
    void signal_flange2tcp_mark_point_received(int index, CartesianPose pose);
    void signal_compute_result_received(double result);
    

public slots:
    void slot_handler_start();                                  // 启动
    void slot_handler_stop();                                   // 停止
    void slot_handler_mark_point(int index);                    // 标定点
    void slot_handler_send_message(QString msg);                // 发送信息给控制器
    void slot_handler_start_record();                           // 开始录制
    void slot_handler_end_record();                             // 结束录制
    void slot_handler_start_playback();                         // 开始回放
    void slot_handler_end_playback();                           // 结束回放
    void slot_handler_flang2tcp_mark_point(int index);          // 法兰盘2TCP的标定点 

private:
    void thread_loop();  // 后台线程处理
    void handle_message(int msg_id, const std::string& msg);
    int serial_id_send_;

    std::thread thread_;
    std::atomic_bool running_{false};
    CommManager comm_;  // 控制器通信管理类（你已有）

};
