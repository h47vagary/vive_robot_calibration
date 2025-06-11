/**
 * @file vive_tracker.h
 * @brief VIVE Tracker 的封装类，管理初始化、姿态获取、记录等功能
 * @version 0.1
 * @date 2025-05-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>
#include <openvr.h>
#include <vector>


namespace htc_vive {

class ViveTracker
{
public:
    ViveTracker();
    ~ViveTracker();

    bool initialize();
    void shutdown();

    bool find_tracker();
    void set_origin(float x, float y, float z,
                    float qx, float qy, float qz, float qw);

    bool get_pose(float& x, float& y, float& z,
                  float& qx, float& qy, float& qz, float& qw,
                  uint64_t& button_mask);

    bool get_pose(double& x, double& y, double& z, 
                  double &A, double &B, double &C,
                  uint64_t& buttom_mask);

    bool get_relative_pose(float& x, float& y, float& z,
                           float& qx, float& qy, float& qz, float& qw,
                           uint64_t& button_mask);


private:
    void quat_inverse(float qx, float qy, float qz, float qw,
                      float& iqx, float& iqy, float& iqz, float& iqw);

    void quat_multiply(float ax, float ay, float az, float aw,
                       float bx, float by, float bz, float bw,
                       float& rx, float& ry, float& rz, float& rw);

    void get_position_and_rotation(const vr::TrackedDevicePose_t& pose,
                                   float& x, float& y, float& z,
                                   float& qx, float& qy, float& qz, float& qw);

    void get_position_and_rotation(const vr::TrackedDevicePose_t& pose,
                                   double& x, double& y, double& z,
                                   double& A, double& B, double& C);

    uint64_t get_button_mask();

private:
    vr::IVRSystem* vr_system_;
    vr::TrackedDeviceIndex_t tracker_index_;
    float origin_x_, origin_y_, origin_z_;
    float origin_qx_, origin_qy_, origin_qz_, origin_qw_;
};

} // namespace htc_vive
