#include "vive_tracker.h"
#include <iostream>
#include <cmath>
#include <thread>
#include "Eigen/Eigen"
#include "utils.h"

namespace htc_vive {

ViveTracker::ViveTracker()
    : vr_system_(nullptr),
      tracker_index_(vr::k_unTrackedDeviceIndexInvalid)
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
    if (error != vr::VRInitError_None) {
        std::cerr << "VR_Init failed: " << vr::VR_GetVRInitErrorAsEnglishDescription(error) << std::endl;
        return false;
    }
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
    if (!vr_system_) return false;
    
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

bool ViveTracker::get_pose(double& x, double& y, double& z,
                           double& qx, double& qy, double& qz, double& qw,
                           uint64_t& button_mask)
{
    if (!vr_system_) 
    {
        std::cerr << __FUNCTION__ << " VR system not initialized" << std::endl;
        return false;
    }

    if (tracker_index_ == vr::k_unTrackedDeviceIndexInvalid)
    {
        std::cerr << __FUNCTION__ << " Tracker not found" << std::endl;
        return false;
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::VRCompositor()->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
    const vr::TrackedDevicePose_t& pose = poses[tracker_index_];
    
    if (!pose.bPoseIsValid) 
    {
        return false;
    }

    get_position_and_rotation(pose, x, y, z, qx, qy, qz, qw);
    
    // Convert from meters to millimeters
    x *= 1000.0f;
    y *= 1000.0f;
    z *= 1000.0f;

    button_mask = get_button_mask();
    return true;
}

bool ViveTracker::get_pose(double &x, double &y, double &z, 
                           double &A, double &B, double &C, 
                           uint64_t &button_mask)
{
    if (!vr_system_) 
    {
        std::cerr << __FUNCTION__ << " VR system not initialized" << std::endl;
        return false;
    }

    if (tracker_index_ == vr::k_unTrackedDeviceIndexInvalid)
    {
        std::cerr << __FUNCTION__ << " Tracker not found" << std::endl;
        return false;
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::VRCompositor()->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
    const vr::TrackedDevicePose_t& pose = poses[tracker_index_];
    
    if (!pose.bPoseIsValid) 
    {
        return false;
    }

    get_position_and_rotation(pose, x, y, z, A, B, C);
    
    // Convert from meters to millimeters
    x *= 1000.0;
    y *= 1000.0;
    z *= 1000.0;

    button_mask = get_button_mask();
    return true;
}

bool ViveTracker::get_pose_non_blocking(double &x, double &y, double &z, double &qx, double &qy, double &qz, double &qw, uint64_t &button_mask)
{
    if (!vr_system_) 
    {
        std::cerr << __FUNCTION__ << " VR system not initialized" << std::endl;
        return false;
    }

    if (tracker_index_ == vr::k_unTrackedDeviceIndexInvalid)
    {
        std::cerr << __FUNCTION__ << " Tracker not found" << std::endl;
        return false;
    }

    float prediction = 0.0f;
    if (vr_system_->IsTrackedDeviceConnected(vr::k_unTrackedDeviceIndex_Hmd)) {
        prediction = vr_system_->GetFloatTrackedDeviceProperty(
            vr::k_unTrackedDeviceIndex_Hmd,
            vr::Prop_SecondsFromVsyncToPhotons_Float
        );
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount] = {};
    vr_system_->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        prediction,
        poses,
        vr::k_unMaxTrackedDeviceCount
    );

    const vr::TrackedDevicePose_t& pose = poses[tracker_index_];
    if (!pose.bPoseIsValid) 
    {
        return false;
    }

    get_position_and_rotation(pose, x, y, z, qx, qy, qz, qw);

    // Convert to millimeters
    x *= 1000.0f;
    y *= 1000.0f;
    z *= 1000.0f;

    button_mask = get_button_mask();
    return true;
}


bool ViveTracker::get_pose_non_blocking(double &x, double &y, double &z, double &A, double &B, double &C, uint64_t &button_mask)
{
    if (!vr_system_) 
    {
        std::cerr << __FUNCTION__ << " VR system not initialized" << std::endl;
        return false;
    }

    if (tracker_index_ == vr::k_unTrackedDeviceIndexInvalid)
    {
        std::cerr << __FUNCTION__ << " Tracker not found" << std::endl;
        return false;
    }

    float prediction = 0.0f;
    if (vr_system_->IsTrackedDeviceConnected(vr::k_unTrackedDeviceIndex_Hmd)) {
        prediction = vr_system_->GetFloatTrackedDeviceProperty(
            vr::k_unTrackedDeviceIndex_Hmd,
            vr::Prop_SecondsFromVsyncToPhotons_Float
        );
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount] = {};
    vr_system_->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        prediction,
        poses,
        vr::k_unMaxTrackedDeviceCount
    );

    const vr::TrackedDevicePose_t& pose = poses[tracker_index_];
    if (!pose.bPoseIsValid) 
    {
        return false;
    }

    get_position_and_rotation(pose, x, y, z, A, B, C);

    // Convert to millimeters
    x *= 1000.0;
    y *= 1000.0;
    z *= 1000.0;

    button_mask = get_button_mask();
    return true;
}

void ViveTracker::quat_inverse(double qx, double qy, double qz, double qw,
                               double& iqx, double& iqy, double& iqz, double& iqw) const
{
    iqx = -qx;
    iqy = -qy;
    iqz = -qz;
    iqw = qw;
}

void ViveTracker::quat_multiply(double ax, double ay, double az, double aw,
                                double bx, double by, double bz, double bw,
                                double& rx, double& ry, double& rz, double& rw) const
{
    rw = aw * bw - ax * bx - ay * by - az * bz;
    rx = aw * bx + ax * bw + ay * bz - az * by;
    ry = aw * by - ax * bz + ay * bw + az * bx;
    rz = aw * bz + ax * by - ay * bx + az * bw;
}

void ViveTracker::get_position_and_rotation(const vr::TrackedDevicePose_t& pose,
                                            double& x, double& y, double& z,
                                            double& qx, double& qy, double& qz, double& qw) const
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

void ViveTracker::get_position_and_rotation(const vr::TrackedDevicePose_t &pose, 
                                           double &x, double &y, double &z, 
                                           double &A, double &B, double &C) const
{
    const vr::HmdMatrix34_t& mat = pose.mDeviceToAbsoluteTracking;
    x = mat.m[0][3];
    y = mat.m[1][3];
    z = mat.m[2][3];

    Eigen::Matrix3d eig_mat = Eigen::Matrix3d::Identity();
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            eig_mat(row, col) = mat.m[row][col];
        }
    }
    Utils::matrix_to_eular_ABC(eig_mat, A, B, C);
}

uint64_t ViveTracker::get_button_mask() const
{
    if (!vr_system_ || tracker_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return 0;

    vr::VRControllerState_t state;
    if (vr_system_->GetControllerState(tracker_index_, &state, sizeof(state)))
    {
        return state.ulButtonPressed;
    }
    return 0;
}

bool ViveTracker::is_tracker_connected() const
{
    return vr_system_ && 
           tracker_index_ != vr::k_unTrackedDeviceIndexInvalid && 
           vr_system_->IsTrackedDeviceConnected(tracker_index_);
}

} // namespace htc_vive