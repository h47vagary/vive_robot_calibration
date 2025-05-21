#include "vive_tracker.h"
#include <iostream>
#include <cmath>
#include <thread>
#include "Eigen/Eigen"
#include "utils.h"

namespace htc_vive {

ViveTracker::ViveTracker()
    : vr_system_(nullptr),
      tracker_index_(vr::k_unTrackedDeviceIndexInvalid),
      origin_x_(0), origin_y_(0), origin_z_(0),
      origin_qx_(0), origin_qy_(0), origin_qz_(0), origin_qw_(1)
{
}

ViveTracker::~ViveTracker()
{
    shutdown();
}

bool ViveTracker::initialize()
{
    vr::EVRInitError error = vr::VRInitError_None;
    vr_system_ = vr::VR_Init(&error, vr::VRApplication_Scene);
    if (error != vr::VRInitError_None) return false;
    return vr::VRCompositor() != nullptr;
}

void ViveTracker::shutdown()
{
    if (vr_system_)
    {
        vr::VR_Shutdown();
        vr_system_ = nullptr;
    }
}

bool ViveTracker::find_tracker()
{
    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
    {
        if (!vr_system_->IsTrackedDeviceConnected(i)) continue;
        if (vr_system_->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_GenericTracker)
        {
            tracker_index_ = i;
            return true;
        }
    }
    return false;
}

void ViveTracker::set_origin(float x, float y, float z,
                             float qx, float qy, float qz, float qw)
{
    origin_x_ = x;
    origin_y_ = y;
    origin_z_ = z;
    origin_qx_ = qx;
    origin_qy_ = qy;
    origin_qz_ = qz;
    origin_qw_ = qw;
}

bool ViveTracker::get_pose(float& x, float& y, float& z,
                           float& qx, float& qy, float& qz, float& qw,
                           uint64_t& button_mask)
{
    if (!vr_system_) 
    {
        std::cout << __FUNCTION__ << " !vr_system_" << std::endl;
        return false;
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::VRCompositor()->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
    const vr::TrackedDevicePose_t& pose = poses[tracker_index_];
    if (!pose.bPoseIsValid) return false;

    get_position_and_rotation(pose, x, y, z, qx, qy, qz, qw);
    
    x *= 1000.0f;
    y *= 1000.0f;
    z *= 1000.0f;

    button_mask = get_button_mask();
    return true;
}

bool ViveTracker::get_pose(double &x, double &y, double &z, double &A, double &B, double &C, uint64_t &button_mask)
{
    if (!vr_system_) 
    {
        std::cout << __FUNCTION__ << " !vr_system_" << std::endl;
        return false;
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::VRCompositor()->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
    const vr::TrackedDevicePose_t& pose = poses[tracker_index_];
    if (!pose.bPoseIsValid) return false;

    get_position_and_rotation(pose, x, y, z, A, B, C);
    
    x *= 1000.0f;
    y *= 1000.0f;
    z *= 1000.0f;

    button_mask = get_button_mask();
    return true;
}

bool ViveTracker::get_relative_pose(float& x, float& y, float& z,
                                    float& qx, float& qy, float& qz, float& qw,
                                    uint64_t& button_mask)
{
    if (!vr_system_)
    {
        std::cout << __FUNCTION__ << " !vr_system_" << std::endl;
        return false;
    }
    
    float abs_x, abs_y, abs_z, abs_qx, abs_qy, abs_qz, abs_qw;
    if (!get_pose(abs_x, abs_y, abs_z, abs_qx, abs_qy, abs_qz, abs_qw, button_mask))
    {
        return false;
    }


    x = (abs_x - origin_x_);
    y = (abs_y - origin_y_);
    z = (abs_z - origin_z_);

    float inv_qx, inv_qy, inv_qz, inv_qw;
    quat_inverse(origin_qx_, origin_qy_, origin_qz_, origin_qw_,
                 inv_qx, inv_qy, inv_qz, inv_qw);
    quat_multiply(inv_qx, inv_qy, inv_qz, inv_qw,
                  abs_qx, abs_qy, abs_qz, abs_qw,
                  qx, qy, qz, qw);

    if (recording_)
    {
        PoseData pose_tmp(x, y, z, qx, qy, qz, qw, button_mask);
        pose_vector_.push_back(pose_tmp);
    }

    return true;
}

void ViveTracker::start_logging(const std::string& filename)
{
    if (recording_)
    {
        std::cout << "please stop during the recording first" << std::endl;
        return;
    }
    pose_vector_.clear();
    recording_ = true;
    filename_ = filename;
}

void ViveTracker::stop_logging()
{
    log_file_.open(filename_);
    if (!log_file_.is_open()) return;

    std::cout << "pose size: " << pose_vector_.size() << std::endl;
    log_file_ << "X(mm),Y(mm),Z(mm),Qx,Qy,Qz,Qw,ButtonPressed" << std::endl;
    for (auto iter : pose_vector_)
    {
        log_file_ << iter.x << "," << iter.y << "," << iter.z << ","
                  << iter.qx << "," << iter.qy << "," << iter.qz << "," << iter.qw << ","
                  << iter.button_mask << std::endl;
    }
    pose_vector_.clear();
    log_file_.close();

    recording_ = false; 
}

void ViveTracker::quat_inverse(float qx, float qy, float qz, float qw,
                               float& iqx, float& iqy, float& iqz, float& iqw)
{
    iqx = -qx;
    iqy = -qy;
    iqz = -qz;
    iqw = qw;
}

void ViveTracker::quat_multiply(float ax, float ay, float az, float aw,
                                float bx, float by, float bz, float bw,
                                float& rx, float& ry, float& rz, float& rw)
{
    rw = aw * bw - ax * bx - ay * by - az * bz;
    rx = aw * bx + ax * bw + ay * bz - az * by;
    ry = aw * by - ax * bz + ay * bw + az * bx;
    rz = aw * bz + ax * by - ay * bx + az * bw;
}

void ViveTracker::get_position_and_rotation(const vr::TrackedDevicePose_t& pose,
                                            float& x, float& y, float& z,
                                            float& qx, float& qy, float& qz, float& qw)
{
    const vr::HmdMatrix34_t& mat = pose.mDeviceToAbsoluteTracking;
    x = mat.m[0][3];
    y = mat.m[1][3];
    z = mat.m[2][3];

    qw = sqrt(fmax(0, 1 + mat.m[0][0] + mat.m[1][1] + mat.m[2][2])) / 2;
    qx = sqrt(fmax(0, 1 + mat.m[0][0] - mat.m[1][1] - mat.m[2][2])) / 2;
    qy = sqrt(fmax(0, 1 - mat.m[0][0] + mat.m[1][1] - mat.m[2][2])) / 2;
    qz = sqrt(fmax(0, 1 - mat.m[0][0] - mat.m[1][1] + mat.m[2][2])) / 2;

    qx = copysign(qx, mat.m[2][1] - mat.m[1][2]);
    qy = copysign(qy, mat.m[0][2] - mat.m[2][0]);
    qz = copysign(qz, mat.m[1][0] - mat.m[0][1]);
}

void ViveTracker::get_position_and_rotation(const vr::TrackedDevicePose_t &pose, double &x, double &y, double &z, double &A, double &B, double &C)
{
    const vr::HmdMatrix34_t& mat = pose.mDeviceToAbsoluteTracking;
    x = mat.m[0][3];
    y = mat.m[1][3];
    z = mat.m[2][3];

    Eigen::Matrix3d eig_mat = Eigen::Matrix3d::Identity();
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col <3; ++col)
        {
            eig_mat(row, col) = mat.m[row][col];
        }
    }
    Utils::matrix_to_eular_ABC(eig_mat, A, B, C);
}

uint64_t ViveTracker::get_button_mask()
{
    vr::VRControllerState_t state;
    if (vr_system_ && vr_system_->GetControllerState(tracker_index_, &state, sizeof(state)))
    {
        return state.ulButtonPressed;
    }
    return 0;
}

} // namespace htc_vive


// 枪按键           HTC VIVE IO                                         枪功能
// red key  : Touchpad  k_EButton_SteamVR_Touchpad                    
// green key : Trigger（扳机） k_EButton_SteamVR_Trigger            喷枪IO信号记录按钮
// blue key : Grip / 手柄握持  k_EButton_Grip                       轨迹记录开始/停止按钮