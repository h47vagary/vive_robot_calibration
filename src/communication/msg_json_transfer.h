/**
 * @file msg_json_transfer.h
 * @brief 通讯关键字和json序列化组件
 * @version 0.1
 * @date 2025-05-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <string>
#include <map>
#include "utils.h"


enum E_JSON_COMMAND
{
    E_JSON_COMMAND_GET_ALL_INFO_ = 300,
    E_JSON_COMMAND_RECEIVE_ALL_INFO_,

    E_JSON_COMMAND_SET_MARK_CALIB_POINT_ = 400,         // 标定点
    E_JSON_COMMAND_SET_COMPUTE_,                        // 开始计算
    E_JSON_COMMAND_SET_DELETE_CALIB_RESULT_,            // 删除标定结果
    E_JSON_COMMAND_SET_SAVE_CALIB_RESULT_,              // 保存标定结果
    E_JSON_COMMAND_SET_SEND_DIY_,                       // 发送自定义命令
    E_JSON_COMMAND_SET_START_RECORD_,                   // 开始录制
    E_JSON_COMMAND_SET_END_RECORD_,                     // 结束录制
    E_JSON_COMMAND_SET_START_PLAYBACK_,                 // 开始回放
    E_JSON_COMMAND_SET_END_PLAYBACK_,                   // 结束回放
    E_JSON_COMMAND_SET_FLANG2TCP_MARK_POINT_,           // 标定法兰盘2TCP点
    E_JSON_COMMAND_SET_TRACKER2TCP_ROTATION_,           // 标定追踪器到TCP点（需要取一次当前机器人法兰盘点位）
    E_JSON_COMMAND_SET_LINEAR_ERROR_USE_ROBOT_POSE_,    // 计算线性误差（需要取一次当前机器人法兰盘点位）
    
    E_JSON_COMMAND_RECEIVE_MES_,                        // 接受控制器消息
    E_JSON_COMMAND_RECEIVE_ROBOT_MARK_POINT_,           // 接受控制器标定点 
    E_JSON_COMMAND_RECEIVE_ROBOT_COMPUTE_RESULT_,       // 接受控制器计算结果
    E_JSON_COMMAND_RECEIVE_FLANG2TCP_MARK_POINT_,       // 接收法兰盘2TCP点
    E_JSON_COMMAND_RECEIVE_TRACKER2TCP_ROTATION_,       // 接收用于标定追踪器到TCP点的当前机器人点位
    E_JOSN_COMMAND_RECEIVE_LINEAR_ERROR_USE_ROBOT_POSE_, // 接收用于计算线性误差的当前机器人点位
};


//结构体转为json 发命令
namespace MsgStructTransfer
{
    int transfer_command(E_JSON_COMMAND user_command, int serial_id, std::string &json_str_out);
    int transfer_mark_point(E_JSON_COMMAND user_command, int serial_id, int point_index, std::string &json_str_out);
    int transfer_rob_pose(E_JSON_COMMAND user_command, int serial_id, int point_index, CartesianPose pose, std::string &json_str_out);
    int transfer_diy_message(E_JSON_COMMAND user_command, int serial_id, const std::string &message, std::string &json_str_out);
    int transfer_robot_compute_result(E_JSON_COMMAND user_command, int serial_id, double result, std::string &json_str_out);

}   // namespace MsgStructTransfer

//json转为结构体 读数据
namespace MsgJsonTransfer
{
    int transfer_command(const std::string &json_str_in,E_JSON_COMMAND &command_out ,int &serial_id_out);
    int transfer_mark_point(const std::string &json_str_in, int &point_index);
    int transfer_rob_pose(const std::string &json_str_in, int &point_index, CartesianPose &pose);
    int transfer_diy_message(const std::string &json_str_in, std::string &message_out);
    int transfer_robot_compute_result(const std::string &json_str_in, double &result_out);
    
}   // namespace MsgJsonTransfer