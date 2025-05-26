#include "message_handler.h"
#include <iostream>
#include "msg_json_transfer.h"

MessageHandler::MessageHandler(QObject* parent)
    : QObject(parent)
{
    serial_id_send_ = 0;
    running_ = false;
}

MessageHandler::~MessageHandler()
{
    stop();
}

void MessageHandler::start()
{
    if (running_)
    {
        std::cout << "[MessageHandler] start() called but thread already running." << std::endl;
        return;
    }

    running_ = true;
    try
    {
        thread_ = std::thread(&MessageHandler::thread_loop, this);
        std::cout << "[MessageHandler] Thread started." << std::endl;
    }
    catch (const std::system_error& e)
    {
        std::cout << "[MessageHandler] Failed to start thread: " << e.what() << std::endl;
        running_ = false;
    }
}

void MessageHandler::stop()
{
    if (!running_)
    {
        std::cout << "[MessageHandler] stop() called but thread not running." << std::endl;
        return;
    }

    running_ = false;
    comm_.nrc_shutdown();

    if (thread_.joinable())
    {
        thread_.join();
        std::cout << "[MessageHandler] Thread stopped." << std::endl;
    }
    else
    {
        std::cout << "[MessageHandler] Thread not joinable on stop." << std::endl;
    }
}

void MessageHandler::thread_loop()
{
    // 1. 初始化通信连接
    if (!comm_.nrc_init("192.168.1.13", "6001"))
    {
        std::cout << "[MessageHandler] Failed to connect to controller" << std::endl;
        return;
    }

    std::cout << "[MessageHandler] Connected to controller" << std::endl;

    // 2. 注册消息接收回调（在通信线程内部）
    comm_.nrc_register_callback([this](int msg_id, const std::string& msg){
        this->handle_message(msg_id, msg);  // 内部解析
    });

    // 3. 发送问候消息
    comm_.nrc_send_message(0x9206, "{\"type\":\"hello\",\"data\":\"Hi NRC please success\"}");

    // 4. 等待终止（线程维持）
    while (running_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MessageHandler::handle_message(int msg_id, const std::string& msg)
{
    if (msg_id != 0x9208) return;

    E_JSON_COMMAND command_out;
    int serial_id_out, tmp_key;
    
    tmp_key = MsgJsonTransfer::transfer_command(msg, command_out, serial_id_out);
    if (0 != tmp_key)
    {
        std::cout << "0 != tmp_key" << std::endl;
        return;
    }

    switch (command_out)
    {
    case E_JSON_COMMAND_RECEIVE_ROBOT_MARK_POINT_:
    {
        //std::cout << "=== 接收当前机器人标定点坐标 ===" << std::endl;
        CartesianPose pose;
        int point;
        if (!MsgJsonTransfer::transfer_rob_pose(msg, point, pose))
        {
            // std::cout << "point: " << point << std::endl;
            // std::cout << "pose.pos.x: " << pose.position.x << "  pose.pos.y: " << pose.position.y << " pose.pos.z: " << pose.position.z
            //             << " pose.ori.A: " << pose.orientation.A << " pose.ori.B: " << pose.orientation.B << "pose.ori.C: " << pose.orientation.C
            //                 << std::endl;
            emit signal_mark_point_received(E_POSE_TYPE_ROBOT, point, pose);
        }
        else
        {
            std::cout << "transfer_rob_pose fail !" << std::endl;
        }
        break;
    }
    case E_JSON_COMMAND_RECEIVE_MES_:
    {
        //std::cout << "=== 接收控制器消息 ===" << std::endl;
        std::string message_out;
        if (!MsgJsonTransfer::transfer_diy_message(msg, message_out))
        {
            std::cout << "message_out: " << message_out << std::endl;
        }
        else
        {
            std::cout << "transfer_diy_message fail !" << std::endl;
        }
        emit signal_message_received(QString::fromStdString(message_out));
        break;
    }
    case E_JSON_COMMAND_RECEIVE_ROBOT_COMPUTE_RESULT_:
    {
        //std::cout << "=== 接收标定计算结果 ===" << std::endl;
        double result_out;
        if (!MsgJsonTransfer::transfer_robot_compute_result(msg, result_out))
        {
            std::cout << "result_out: " << result_out << std::endl;
            emit signal_compute_result_received(result_out);
        }
        else
        {
            std::cout << "transfer_robot_compute_result fail !" << std::endl;
        }
        break;
    }
    case E_JSON_COMMAND_RECEIVE_FLANG2TCP_MARK_POINT_:
    {
        //std::cout << "=== 接收法兰盘->TCP标定点坐标 ===" << std::endl;
        CartesianPose pose;
        int point;
        if (!MsgJsonTransfer::transfer_rob_pose(msg, point, pose))
        {
            // std::cout << "point: " << point << std::endl;
            // std::cout << "pose.pos.x: " << pose.position.x << "  pose.pos.y: " << pose.position.y << " pose.pos.z: " << pose.position.z
            //             << " pose.ori.A: " << pose.orientation.A << " pose.ori.B: " << pose.orientation.B << "pose.ori.C: " << pose.orientation.C
            //                 << std::endl;
            emit signal_flange2tcp_mark_point_received(point, pose);
        }
        else
        {
            std::cout << "transfer_rob_pose fail !" << std::endl;
        }
        break;
    }
    case E_JSON_COMMAND_RECEIVE_TRACKER2TCP_ROTATION_:
    {
        std::cout << "######## E_JSON_COMMAND_RECEIVE_TRACKER2TCP_ROTATION_ #############" << std::endl;
        CartesianPose pose;
        int point;
        if (!MsgJsonTransfer::transfer_rob_pose(msg, point, pose))
        {
            // std::cout << "point: " << point << std::endl;
            // std::cout << "pose.pos.x: " << pose.position.x << "  pose.pos.y: " << pose.position.y << " pose.pos.z: " << pose.position.z
            //             << " pose.ori.A: " << pose.orientation.A << " pose.ori.B: " << pose.orientation.B << "pose.ori.C: " << pose.orientation.C
            //                 << std::endl;
            emit signal_tracker2tcp_mark_use_robot_pose(pose);
        }
        else
        {
            std::cout << "transfer_rob_pose fail !" << std::endl;
        }
        break;
    }
    case E_JOSN_COMMAND_RECEIVE_LINEAR_ERROR_USE_ROBOT_POSE_:
    {
        CartesianPose pose;
        int point;
        if (!MsgJsonTransfer::transfer_rob_pose(msg, point, pose))
        {
            // std::cout << "point: " << point << std::endl;
            // std::cout << "pose.pos.x: " << pose.position.x << "  pose.pos.y: " << pose.position.y << " pose.pos.z: " << pose.position.z
            //             << " pose.ori.A: " << pose.orientation.A << " pose.ori.B: " << pose.orientation.B << "pose.ori.C: " << pose.orientation.C
            //                 << std::endl;
            emit signal_get_linear_error_use_robot_pose(pose);
        }
        else
        {
            std::cout << "transfer_rob_pose fail !" << std::endl;
        }
    }
    default:
    {
        //std::cout << "unknown command !" << std::endl;
        break;
    }

    }
    return;
}

void MessageHandler::slot_handler_start()
{
    this->start();
}

void MessageHandler::slot_handler_stop()
{
    this->stop();
}

void MessageHandler::slot_handler_mark_point(int index)
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_mark_point(E_JSON_COMMAND_SET_MARK_CALIB_POINT_, ++serial_id_send_, index, json_str_out);
    comm_.nrc_send_message(0x9206, json_str_out);
}

void MessageHandler::slot_handler_send_message(QString msg)
{
    if (!running_) return;
    std::cout << __FUNCTION__ << " msg: " << msg.toStdString() << std::endl;
    comm_.nrc_send_message(0x9206, msg.toStdString());
}

void MessageHandler::slot_handler_start_record()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_START_RECORD_, ++serial_id_send_, json_str_out);
    //std::cout << "json_str_out: " << json_str_out << std::endl;
    comm_.nrc_send_message(0x9206, json_str_out);
}

void MessageHandler::slot_handler_end_record()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_END_RECORD_, ++serial_id_send_, json_str_out);
    //std::cout << "json_str_out: " << json_str_out << std::endl;
    comm_.nrc_send_message(0x9206, json_str_out);
}

void MessageHandler::slot_handler_start_playback()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_START_PLAYBACK_, ++serial_id_send_, json_str_out);
    //std::cout << "json_str_out: " << json_str_out << std::endl;
    comm_.nrc_send_message(0x9206, json_str_out);
}

void MessageHandler::slot_handler_end_playback()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_END_PLAYBACK_, ++serial_id_send_, json_str_out);
    //std::cout << "json_str_out: " << json_str_out << std::endl;
    comm_.nrc_send_message(0x9206, json_str_out);
}

void MessageHandler::slot_handler_flang2tcp_mark_point(int index)
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_mark_point(E_JSON_COMMAND_SET_FLANG2TCP_MARK_POINT_, ++serial_id_send_, index, json_str_out);
    //std::cout << "json_str_out: " << json_str_out << std::endl;
    comm_.nrc_send_message(0x9206, json_str_out);
}

void MessageHandler::slot_handler_tracker2tcp_mark_rotation_use_robotpose()
{
    std::cout << __FUNCTION__ << std::endl;
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_TRACKER2TCP_ROTATION_, ++serial_id_send_, json_str_out);
    comm_.nrc_send_message(0x9206, json_str_out);
}

void MessageHandler::slot_linear_error_acquire()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_LINEAR_ERROR_USE_ROBOT_POSE_, ++serial_id_send_, json_str_out);
    comm_.nrc_send_message(0x9206, json_str_out);
}
