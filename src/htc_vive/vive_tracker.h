/**
 * @file vive_tracker.h
 * @brief VIVE Tracker 的封装类，管理初始化、姿态获取等功能
 * @version 0.2
 * @date 2025-05-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <cstdint>
#include <openvr.h>

namespace htc_vive {

class ViveTracker
{
public:
    ViveTracker();
    ~ViveTracker();

    /**
     * @brief 初始化VR系统
     * @return 初始化成功返回true，失败返回false
     */
    bool initialize();

    /**
     * @brief 关闭VR系统
     */
    void shutdown();

    /**
     * @brief 查找连接的Vive Tracker设备
     * @return 找到设备返回true，否则返回false
     */
    bool find_tracker();

    /**
     * @brief 阻塞式获获取追踪器的姿态信息(四元数表示)
     * @note  使用 WaitGetPoses() 会等待 vr::Compositor 的下一帧 Sync 数据
     * @param[out] x 位置的x坐标(mm)
     * @param[out] y 位置的y坐标(mm)
     * @param[out] z 位置的z坐标(mm)
     * @param[out] qx 旋转的四元数x分量
     * @param[out] qy 旋转的四元数y分量
     * @param[out] qz 旋转的四元数z分量
     * @param[out] qw 旋转的四元数w分量
     * @param[out] button_mask 按钮状态掩码
     * @return 获取成功返回true，失败返回false
     */
    bool get_pose(float& x, float& y, float& z,
                  float& qx, float& qy, float& qz, float& qw,
                  uint64_t& button_mask);

    /**
     * @brief 阻塞式获取追踪器的姿态信息(欧拉角表示)
     * @note  使用 WaitGetPoses() 会等待 vr::Compositor 的下一帧 Sync 数据
     * @param[out] x 位置的x坐标(mm)
     * @param[out] y 位置的y坐标(mm)
     * @param[out] z 位置的z坐标(mm)
     * @param[out] A 欧拉角A(弧度)
     * @param[out] B 欧拉角B(弧度)
     * @param[out] C 欧拉角C(弧度)
     * @param[out] button_mask 按钮状态掩码
     * @return 获取成功返回true，失败返回false
     */
    bool get_pose(double& x, double& y, double& z, 
                  double& A, double& B, double& C,
                  uint64_t& button_mask);

    /**
     * @brief 非阻塞式获取姿态信息(四元数)
     * @note 使用 GetDeviceToAbsoluteTrackingPose() 立即返回当前数据
     */
    bool get_pose_non_blocking(float& x, float& y, float& z,
                              float& qx, float& qy, float& qz, float& qw,
                              uint64_t& button_mask);

    /**
     * @brief 非阻塞式获取姿态信息(欧拉角)
     * @note 使用 GetDeviceToAbsoluteTrackingPose() 立即返回当前数据
     */
    bool get_pose_non_blocking(double& x, double& y, double& z,
                              double& A, double& B, double& C,
                              uint64_t& button_mask);

    /**
     * @brief 检查追踪器是否已连接
     * @return 已连接返回true，否则返回false
     */
    bool is_tracker_connected() const;

private:
    /**
     * @brief 计算四元数的逆
     */
    void quat_inverse(float qx, float qy, float qz, float qw,
                      float& iqx, float& iqy, float& iqz, float& iqw) const;

    /**
     * @brief 四元数乘法
     */
    void quat_multiply(float ax, float ay, float az, float aw,
                       float bx, float by, float bz, float bw,
                       float& rx, float& ry, float& rz, float& rw) const;

    /**
     * @brief 从姿态矩阵提取位置和四元数旋转
     */
    void get_position_and_rotation(const vr::TrackedDevicePose_t& pose,
                                   float& x, float& y, float& z,
                                   float& qx, float& qy, float& qz, float& qw) const;

    /**
     * @brief 从姿态矩阵提取位置和欧拉角旋转
     */
    void get_position_and_rotation(const vr::TrackedDevicePose_t& pose,
                                   double& x, double& y, double& z,
                                   double& A, double& B, double& C) const;

    /**
     * @brief 获取当前按钮状态掩码
     * @return 按钮状态掩码
     */
    uint64_t get_button_mask() const;

private:
    vr::IVRSystem* vr_system_;
    vr::TrackedDeviceIndex_t tracker_index_;
};

} // namespace htc_vive