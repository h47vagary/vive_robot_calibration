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

bool MessageHandler::handle_pose_with_point(const std::string &msg, std::function<void(int, const CartesianPose &)> emitter)
{
    CartesianPose pose;
    int point;
    if (!MsgJsonTransfer::transfer_rob_pose(msg, point, pose))
    {
        emitter(point, pose);
        return true;
    }
    std::cout << "transfer_rob_pose fail !" << std::endl;
    return false;
}

void MessageHandler::thread_loop()
{
    if (!comm_.nrc_init(CONTROLLER_IP, CONTROLLER_PORT))
    {
        std::cout << "[MessageHandler] Failed to connect to controller" << std::endl;
        return;
    }
    std::cout << "[MessageHandler] Connected to controller" << std::endl;

    // 注册消息接收回调
    comm_.nrc_register_callback([this](int msg_id, const std::string& msg){
        this->handle_message(msg_id, msg);
    });

    while (running_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MessageHandler::handle_message(int msg_id, const std::string& msg)
{
    if (msg_id != CONTROLLER_RECEIVE_ID) return;

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
        handle_pose_with_point(msg, [this](int index, const CartesianPose& pose) {
            emit signal_mark_point_received(index, pose);
        });
        break;
    }
    case E_JSON_COMMAND_RECEIVE_ROBOT_COMPUTE_RESULT_:
    {
        double result_out;
        if (!MsgJsonTransfer::transfer_robot_compute_result(msg, result_out))
        {
            std::cout << "result_out: " << result_out << std::endl;
            emit signal_compute_result_received(result_out);
        }
        else
            std::cout << "transfer_robot_compute_result fail !" << std::endl;
        break;
    }
    case E_JSON_COMMAND_RECEIVE_FLANG2TCP_MARK_POINT_:
    {
        handle_pose_with_point(msg, [this](int index, const CartesianPose& pose) {
            emit signal_flange2tcp_mark_point_received(index, pose);
        });
        break;
    }
    case E_JSON_COMMAND_RECEIVE_TRACKER2TCP_ROTATION_:
    {
        handle_pose_with_point(msg, [this](int index, const CartesianPose& pose) {
            emit signal_tracker2tcp_mark_use_robot_pose(pose);
        });
        break;
    }
    case E_JSON_COMMAND_RECEIVE_LINEAR_ERROR_USE_ROBOT_POSE_:
    {
        handle_pose_with_point(msg, [this](int index, const CartesianPose& pose) {
            emit signal_get_linear_error_use_robot_pose(pose);
        });
        break;
    }
    default:
    {
        std::cout << "unknown command !" << std::endl;
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
    comm_.nrc_send_message(CONTROLLER_SERIAL_ID, json_str_out);
}

void MessageHandler::slot_handler_start_record()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_START_RECORD_, ++serial_id_send_, json_str_out);
    comm_.nrc_send_message(CONTROLLER_SERIAL_ID, json_str_out);
}

void MessageHandler::slot_handler_end_record()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_END_RECORD_, ++serial_id_send_, json_str_out);
    comm_.nrc_send_message(CONTROLLER_SERIAL_ID, json_str_out);
}

void MessageHandler::slot_handler_start_playback()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_START_PLAYBACK_, ++serial_id_send_, json_str_out);
    comm_.nrc_send_message(CONTROLLER_SERIAL_ID, json_str_out);
}

void MessageHandler::slot_handler_end_playback()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_END_PLAYBACK_, ++serial_id_send_, json_str_out);
    comm_.nrc_send_message(CONTROLLER_SERIAL_ID, json_str_out);
}

void MessageHandler::slot_handler_flang2tcp_mark_point(int index)
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_mark_point(E_JSON_COMMAND_SET_FLANG2TCP_MARK_POINT_, ++serial_id_send_, index, json_str_out);
    comm_.nrc_send_message(CONTROLLER_SERIAL_ID, json_str_out);
}

void MessageHandler::slot_handler_tracker2tcp_mark_rotation_use_robotpose()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_TRACKER2TCP_ROTATION_, ++serial_id_send_, json_str_out);
    comm_.nrc_send_message(CONTROLLER_SERIAL_ID, json_str_out);
}

void MessageHandler::slot_linear_error_acquire()
{
    if (!running_) return;
    std::string json_str_out;
    MsgStructTransfer::transfer_command(E_JSON_COMMAND_SET_LINEAR_ERROR_USE_ROBOT_POSE_, ++serial_id_send_, json_str_out);
    comm_.nrc_send_message(CONTROLLER_SERIAL_ID, json_str_out);
}
