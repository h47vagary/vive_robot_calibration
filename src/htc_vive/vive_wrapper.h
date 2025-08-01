/**
 * @file vive_wrapper.h
 * @brief 封装了 VIVE Tracker 功能接口，开发者可直接使用对应功能接口
 * @version 0.1
 * @date 2025-05-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 VIVE Tracker 系统。
 *
 * 初始化 OpenVR 系统并准备好进行 Tracker 的数据获取。
 * 
 * @return true 初始化成功，false 失败。
 */
bool vive_initialize();

/**
 * @brief 关闭 VIVE Tracker 系统。
 *
 * 释放资源并关闭 OpenVR 系统。
 */
void vive_shutdown();

/**
 * @brief 查找系统中连接的 VIVE Tracker。
 *
 * 扫描所有 OpenVR 设备并找到第一个 Generic Tracker 类型的设备。
 *
 * @return true 找到 Tracker，false 未找到。
 */
bool vive_find_tracker();

/**
 * @brief 阻塞式获取当前 Tracker 的全局位置与姿态（四元数表示）。
 *
 * 使用 WaitGetPoses() 会等待下一帧数据
 * 
 * @param[out] x 位置 X（单位：米）
 * @param[out] y 位置 Y（单位：米）
 * @param[out] z 位置 Z（单位：米）
 * @param[out] qx 姿态四元数 X 分量
 * @param[out] qy 姿态四元数 Y 分量
 * @param[out] qz 姿态四元数 Z 分量
 * @param[out] qw 姿态四元数 W 分量
 * @param[out] button_mask 当前按键状态（bitmask）
 *
 * @return true 获取成功，false 获取失败或 Tracker 姿态无效。
 */
bool vive_get_pose_quaternion(double* x, double* y, double* z,
                   double* qx, double* qy, double* qz, double* qw,
                   uint64_t* button_mask);

/**
 * @brief 阻塞式获取当前 Tracker 的全局位置与姿态（欧拉角表示）。
 *
 * 使用 WaitGetPoses() 会等待下一帧数据
 * 
 * @param[out] x 位置 X（单位：米）
 * @param[out] y 位置 Y（单位：米）
 * @param[out] z 位置 Z（单位：米）
 * @param[out] A 姿态四元数 X 分量
 * @param[out] B 姿态四元数 Y 分量
 * @param[out] C 姿态四元数 Z 分量
 * @param[out] button_mask 当前按键状态（bitmask）
 *
 * @return true 获取成功，false 获取失败或 Tracker 姿态无效。
 */
bool vive_get_pose_euler(double* x, double* y, double* z,
                   double* A, double* B, double* C,
                   uint64_t* button_mask);

/**
 * @brief 非阻塞式获取当前 Tracker 的全局位置与姿态（四元数表示）。
 *
 * 使用 GetDeviceToAbsoluteTrackingPose() 立即返回当前数据
 *
 * @param[out] x 位置 X（单位：米）
 * @param[out] y 位置 Y（单位：米）
 * @param[out] z 位置 Z（单位：米）
 * @param[out] qx 姿态四元数 X 分量
 * @param[out] qy 姿态四元数 Y 分量
 * @param[out] qz 姿态四元数 Z 分量
 * @param[out] qw 姿态四元数 W 分量
 * @param[out] button_mask 当前按键状态（bitmask）
 *
 * @return true 获取成功，false 获取失败或 Tracker 姿态无效。
 */
bool vive_get_pose_quaternion_non_blocking(double* x, double* y, double* z,
                               double* qx, double* qy, double* qz, double* qw,
                               uint64_t* button_mask);

/**
 * @brief 非阻塞式获取当前 Tracker 的全局位置与姿态（欧拉角表示）。
 *
 * 使用 GetDeviceToAbsoluteTrackingPose() 立即返回当前数据
 *
 * @param[out] x 位置 X（单位：米）
 * @param[out] y 位置 Y（单位：米）
 * @param[out] z 位置 Z（单位：米）
 * @param[out] A 姿态欧拉角 A（弧度）
 * @param[out] B 姿态欧拉角 B（弧度）
 * @param[out] C 姿态欧拉角 C（弧度）
 * @param[out] button_mask 当前按键状态（bitmask）
 *
 * @return true 获取成功，false 获取失败或 Tracker 姿态无效。
 */
bool vive_get_pose_euler_non_blocking(double* x, double* y, double* z,
                                   double* A, double* B, double* C,
                                   uint64_t* button_mask);


#ifdef __cplusplus
}
#endif
