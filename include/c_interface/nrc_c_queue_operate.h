/*
 * nrc_c_queue_operate.h
 *
 *  Created on: 2025年1月24日
 *      Author: yiixiong
 */

#ifndef INTERFACE_C_INTERFACE_NRC_C_QUEUE_OPERATE_H_
#define INTERFACE_C_INTERFACE_NRC_C_QUEUE_OPERATE_H_

#include "parameter/nrc_define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief 打开or关闭控制器的队列运动模式
* @param status 0-关闭 1-打开
* @return 0-成功 -1-失败 -2-未连接机器人
* @warning 无论是打开还是关闭，都将清空远端控制器已经存储的队列
*/
EXPORT_API int queue_motion_set_status_c(SOCKETFD socketFd, bool status);

/**
* @brief 将本地队列的前size个数据发送到控制器
* @param size 要发送的队列大小
* @return 0-成功 -1-失败 -2-未连接机器人 -3-传入的size超过队列长度
* @warning 控制器接受到队列后，将会立马开始运动。调用前请先调用queue_motion_set_status(true);
*/
EXPORT_API int queue_motion_send_to_controller_c(SOCKETFD socketFd,int size);

/**
 * @brief 暂停连续运动
 */
EXPORT_API int queue_motion_suspend_c(SOCKETFD socketFd);

/**
 * @brief 暂停后再次运行运动队列
 */
EXPORT_API int queue_motion_restart_c(SOCKETFD socketFd);

/**
 * @brief 停止连续运动保持上电状态
 */
EXPORT_API int queue_motion_stop_not_power_off_c(SOCKETFD socketFd);

/**
* @brief 队列运动模式的本地队列最后插入一条moveJ运动
* @param moveCmd
*  targetPos 目标点坐标 长度7位
*  velocity 参数范围(1,100] 度/s
*  acc，dec 参数范围(1,100]
*  pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API int queue_motion_push_back_moveJ_c(SOCKETFD socketFd, int coord, double vel, double acc, double dec, double* targetPos);

/**
* @brief 队列运动模式的本地队列最后插入一条moveL运动
* @param targetPos 目标点坐标 长度7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API int queue_motion_push_back_moveL_c(SOCKETFD socketFd, int coord, double vel, double acc, double dec, double* targetPos);

/**
* @brief 队列运动模式的本地队列最后插入一条DOUT指令
* @param port  端口
* @param value
*/
EXPORT_API int queue_motion_push_back_dout_c(SOCKETFD socketFd, int port, bool value);

#ifdef __cplusplus
}
#endif
#endif /* INTERFACE_C_INTERFACE_NRC_C_QUEUE_OPERATE_H_ */
