#include "utils.h"

#include <unistd.h>
#include <fstream>
#include <string>

static double clamp_angle(double angle_rad, double max_abs_rad)
{
    if (angle_rad > max_abs_rad)
        return max_abs_rad;
    else if (angle_rad < -max_abs_rad)
        return -max_abs_rad;
    else
        return angle_rad;
}

void Utils::euler_ABC_to_matrix(const double &A, const double &B, const double &C, Eigen::Matrix3d &matrix)
{
    matrix =
            Eigen::AngleAxisd(A, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(B, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(C, Eigen::Vector3d::UnitZ());
}

void Utils::quaternion_to_matrix(const CartesianQuaternion &quat, Eigen::Matrix3d &matrix)
{
    Eigen::Quaterniond eigen_quat(quat.qw, quat.qx, quat.qy, quat.qz);
    matrix = eigen_quat.toRotationMatrix();
}

void Utils::matrix_to_eular_ABC(const Eigen::Matrix3d &matrix, double &A, double &B, double &C)
{
    #if 1
    if (fabs(matrix(1, 2)) < EPSILON && fabs(matrix(2, 2)) < EPSILON)
    {
        if (matrix(0, 2) > EPSILON)
        {
            A = atan2(matrix(2, 1), matrix(1, 1));
        }
        else
        {
            A = -atan2(matrix(1, 0), matrix(2, 0));
        }
        C = 0;
    }
    else
    {
        A = atan2(-matrix(1, 2), matrix(2, 2));
        C = atan2(-matrix(0, 1), matrix(0, 0));
    }
    B = atan2(matrix(0, 2), cos(A) * matrix(2, 2)- sin(A) * matrix(1, 2));
    #else
    Eigen::Vector3d euler = matrix.eulerAngles(0, 1, 2); // XYZ 内旋
    A = euler[0];  // X
    B = euler[1];  // Y
    C = euler[2];  // Z
    #endif
}

void Utils::matrix_to_quaternion(const Eigen::Matrix3d &matrix, Quaternion &quat)
{
    Eigen::Quaterniond quaternion(matrix);
    quat.w = quaternion.w();
    quat.x = quaternion.x();
    quat.y = quaternion.y();
    quat.z = quaternion.z();
    quat = quat.normalize();
}

void Utils::matrix_to_quaternion_with_pitch_limit(
    const Eigen::Matrix3d &matrix,
    Quaternion &quat,
    double pitch_limit_deg)
{
    // 1. 矩阵转Eigen四元数
    Eigen::Quaterniond q_eigen(matrix);

    // 2. Eigen四元数转ZYX欧拉角（Yaw-Pitch-Roll）
    Eigen::Vector3d euler_zyx = q_eigen.toRotationMatrix().eulerAngles(2, 1, 0);

    // 3. 限制Pitch角（中间角，索引1）
    double pitch_limit_rad = pitch_limit_deg * M_PI / 180.0;
    euler_zyx[1] = clamp_angle(euler_zyx[1], pitch_limit_rad);

    // 4. 由限制后的欧拉角重新生成四元数
    Eigen::AngleAxisd yawAngle(euler_zyx[0], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(euler_zyx[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(euler_zyx[2], Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_limited = yawAngle * pitchAngle * rollAngle;

    // 5. 转成你的Quaternion类，并归一化
    quat.w = q_limited.w();
    quat.x = q_limited.x();
    quat.y = q_limited.y();
    quat.z = q_limited.z();

    quat = quat.normalize();
}

void Utils::euler_ABC_to_quaternion(const double &A, const double &B, const double &C, Quaternion &quat)
{
    Eigen::Matrix3d matrix;
    euler_ABC_to_matrix(A, B, C, matrix);
    Eigen::Quaterniond quaternion(matrix);
    quat.w = quaternion.w();
    quat.x = quaternion.x();
    quat.y = quaternion.y();
    quat.z = quaternion.z();
    quat = quat.normalize();
}

void Utils::quaternion_to_euler_ABC(const Quaternion &quat, double &A, double &B, double &C)
{
    Eigen::Quaterniond quaternion(quat.w, quat.x, quat.y, quat.z);
    matrix_to_eular_ABC(quaternion.matrix(), A, B, C);
}

void Utils::plerp(const CartesianPosition &position_start, const CartesianPosition &position_end, const double &t, CartesianPosition &position_t)
{
    position_t.x = position_start.x + (position_end.x - position_start.x) * t;
    position_t.y = position_start.y + (position_end.y - position_start.y) * t;
    position_t.z = position_start.z + (position_end.z - position_start.z) * t;
}

void Utils::slerp(const Quaternion &quat_start, const Quaternion &quat_end, const double &t, Quaternion &quat_t)
{
    Quaternion quat_end_tmp = quat_end;
    double dot_product = quat_start.dot(quat_end_tmp);
    if(dot_product < 0)
    {
        quat_end_tmp = -quat_end_tmp;
        dot_product = quat_start.dot(quat_end_tmp);
    }

    if(dot_product > 0.9995)
    {
        quat_t = (quat_start + (quat_end_tmp - quat_start) * t).normalize();
    }
    else
    {
        double theta = acos(dot_product);
        quat_t = (quat_start * sin((1 - t) * theta) / sin(theta) + quat_end_tmp * sin(t * theta) / sin(theta)).normalize();
    }
}

Eigen::Matrix4d Utils::pose_to_matrix(const CartesianPose &pose)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotation;
    Utils::euler_ABC_to_matrix(pose.orientation.A, pose.orientation.B, pose.orientation.C, rotation);

    mat.block<3,3>(0,0) = rotation;
    mat(0, 3) = pose.position.x;
    mat(1, 3) = pose.position.y;
    mat(2, 3) = pose.position.z;

    return mat;
}

CartesianPose Utils::matrix_to_pose(const Eigen::Matrix4d &mat)
{
    CartesianPose pose;
    pose.position.x = mat(0,3);
    pose.position.y = mat(1,3);
    pose.position.z = mat(2,3);
    
    Eigen::Matrix3d rotation = mat.block<3,3>(0,0);
    Utils::matrix_to_eular_ABC(rotation, pose.orientation.A, pose.orientation.B, pose.orientation.C);

    return pose;
}

Eigen::Matrix4d Utils::pose_to_matrix_use_quaternion(const CartesianPose &pose)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotation;

    Utils::quaternion_to_matrix(pose.quat, rotation);

    mat.block<3,3>(0,0) = rotation;
    mat(0, 3) = pose.position.x;
    mat(1, 3) = pose.position.y;
    mat(2, 3) = pose.position.z;

    return mat;
}

CartesianPose Utils::matrix_to_pose_use_quaternion(const Eigen::Matrix4d &mat)
{
    CartesianPose pose;
    pose.position.x = mat(0,3);
    pose.position.y = mat(1,3);
    pose.position.z = mat(2,3);
    Quaternion quat;
    Eigen::Matrix3d rotation = mat.block<3,3>(0,0);
    Utils::matrix_to_quaternion(rotation, quat);
    //Utils::matrix_to_quaternion_with_pitch_limit(rotation, quat);
    pose.quat.qw = quat.w;
    pose.quat.qx = quat.x;
    pose.quat.qy = quat.y;
    pose.quat.qz = quat.z;


    Utils::quaternion_to_euler_ABC(quat, pose.orientation.A, pose.orientation.B, pose.orientation.C);

    return pose;
}

void Utils::print_matrix(const Eigen::Matrix4d &mat)
{
    for (int i = 0; i < 4; ++i)
    {
        std::cout << " [ ";
        for (int j = 0; j < 4; ++j)
        {
            std::cout << mat(i, j);
            if (j < 3) std::cout << ", ";
        }
        std::cout << " ]\n";
    }
}


uint64_t TimeDealUtils::get_timestamp()
{
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();

    // 获取秒级时间戳
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    // 计算毫秒部分
    auto duration = now.time_since_epoch();
    uint64_t milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    uint64_t now_seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    uint64_t millis_part = milliseconds - now_seconds * 1000;

    // 返回完整的 Unix 时间戳（单位：毫秒）
    return static_cast<uint64_t>(now_time_t) * 1000 + millis_part;
}

uint64_t TimeDealUtils::get_time_difference(uint64_t start, uint64_t end)
{
    return (end > start) ? (end - start) : 0;
}

std::string TimeDealUtils::timestamp_to_string(uint64_t timestamp)
{
    std::time_t time = static_cast<std::time_t>(timestamp / 1000);
    std::tm tm_time;

#ifdef _WIN32
    localtime_s(&tm_time, &time);
#else
    localtime_r(&time, &tm_time);
#endif

    char buffer[6];
    std::sprintf(buffer, "%02d:%02d", tm_time.tm_min, tm_time.tm_sec);

    return std::string(buffer);
}

#ifdef _WIN32
bool set_process_high_priority()
{
    HANDLE h_process = GetCurrentProcess();
    if (!SetPriorityClass(h_process, REALTIME_PRIORITY_CLASS))
    {
        std::cerr << "Failed to set process priority. Error: " << GetLastError() << std::endl;
        return false;
    }
    return true;
}

bool set_thread_high_priority()
{
    HANDLE h_thread = GetCurrentThread();
    if (!SetThreadPriority(h_thread, THREAD_PRIORITY_TIME_CRITICAL))
    {
        std::cerr << "Failed to set thread priority. Error: " << GetLastError() << std::endl;
        return false;
    }
    return true;
}
bool bind_thread_to_cpu(int cpu_index)
{
    DWORD_PTR mask = 1ull << cpu_index;
    HANDLE thread = GetCurrentThread();
    if ( !SetThreadAffinityMask(thread, mask))
    {
        std::cerr << "Failed to bind thread to cpu. Error: " << GetLastError() << std::endl;
        return false;
    }
    return true;
}
bool lock_memory_region(void *ptr, size_t size)
{
    return VirtualLock(ptr, size);
}

void unlock_memory_region(void *ptr, size_t size)
{
    VirtualUnlock(ptr, size);
}
#endif