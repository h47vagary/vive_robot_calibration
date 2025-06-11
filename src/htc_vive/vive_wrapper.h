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
 * @brief 设置当前 Tracker 的原点位姿。
 *
 * 后续调用 `vive_get_relative_pose` 获取的是相对于该原点的相对姿态。
 *
 * @param x 原点位置 X（单位：米）
 * @param y 原点位置 Y（单位：米）
 * @param z 原点位置 Z（单位：米）
 * @param qx 四元数 X 分量
 * @param qy 四元数 Y 分量
 * @param qz 四元数 Z 分量
 * @param qw 四元数 W 分量
 */
void vive_set_origin(float x, float y, float z,
                     float qx, float qy, float qz, float qw);

/**
 * @brief 获取当前 Tracker 的全局位置与姿态。
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
bool vive_get_pose(float* x, float* y, float* z,
                   float* qx, float* qy, float* qz, float* qw,
                   uint64_t* button_mask);

/**
 * @brief 获取当前 Tracker 的全局位置与姿态。
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
bool vive_get_pose_abc(double* x, double* y, double* z,
                   double* A, double* B, double* C,
                   uint64_t* button_mask);

/**
 * @brief 获取 Tracker 相对于设置原点的相对位置与姿态。
 *
 * @param[out] x 相对位置 X（单位：毫米）
 * @param[out] y 相对位置 Y（单位：毫米）
 * @param[out] z 相对位置 Z（单位：毫米）
 * @param[out] qx 相对姿态四元数 X 分量
 * @param[out] qy 相对姿态四元数 Y 分量
 * @param[out] qz 相对姿态四元数 Z 分量
 * @param[out] qw 相对姿态四元数 W 分量
 * @param[out] button_mask 当前按键状态（bitmask）
 *
 * @return true 获取成功，false 获取失败或 Tracker 姿态无效。
 */
bool vive_get_relative_pose(float* x, float* y, float* z,
                            float* qx, float* qy, float* qz, float* qw,
                            uint64_t* button_mask);

#ifdef __cplusplus
}
#endif
