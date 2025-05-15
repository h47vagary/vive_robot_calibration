#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>
#include <openvr.h>
#include <vector>


namespace htc_vive {

/**
 * @brief VIVE Tracker 的封装类，管理初始化、姿态获取、记录等功能
 */
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

    bool get_relative_pose(float& x, float& y, float& z,
                           float& qx, float& qy, float& qz, float& qw,
                           uint64_t& button_mask);

    void start_logging(const std::string& filename);
    void stop_logging();

private:
    void quat_inverse(float qx, float qy, float qz, float qw,
                      float& iqx, float& iqy, float& iqz, float& iqw);

    void quat_multiply(float ax, float ay, float az, float aw,
                       float bx, float by, float bz, float bw,
                       float& rx, float& ry, float& rz, float& rw);

    void get_position_and_rotation(const vr::TrackedDevicePose_t& pose,
                                   float& x, float& y, float& z,
                                   float& qx, float& qy, float& qz, float& qw);

    uint64_t get_button_mask();

private:
    vr::IVRSystem* vr_system_;
    vr::TrackedDeviceIndex_t tracker_index_;
    float origin_x_, origin_y_, origin_z_;
    float origin_qx_, origin_qy_, origin_qz_, origin_qw_;

    bool recording_;
    std::string filename_;
    struct PoseData
    {
        float x, y, z, qx, qy, qz, qw;
        uint64_t button_mask;
        PoseData(float x_, float y_,float z_, 
                 float qx_, float qy_, float qz_, float qw_,
                 uint64_t button_mask_)
                : x(x_), y(y_), z(z_), qx(qx_), qy(qy_), qz(qz_), qw(qw_), button_mask(button_mask_) {}   
    };
    std::vector<PoseData> pose_vector_;
    size_t max_pose_size_;
    std::ofstream log_file_;
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace htc_vive
