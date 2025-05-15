#include "utils.h"

#include <unistd.h>
#include <fstream>
#include <string>

void Utils::euler_ABC_to_matrix(const double &A, const double &B, const double &C, Eigen::Matrix3d &matrix)
{
    matrix =
            Eigen::AngleAxisd(A, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(B, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(C, Eigen::Vector3d::UnitZ());
}

void Utils::euler_RPY_to_matrix(const double &roll, const double &pitch, const double &yaw, Eigen::Matrix3d &matrix)
{
    matrix =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

void Utils::matrix_to_eular_ABC(const Eigen::Matrix3d &matrix, double &A, double &B, double &C)
{
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
}

void Utils::matrix_to_eular_RPY(const Eigen::Matrix3d &matrix, double &roll, double &pitch, double &yaw)
{
    yaw = atan2(matrix(1, 0), matrix(0, 0));
    pitch = atan2(-matrix(2, 0), cos(yaw) * matrix(0, 0) + sin(yaw) * matrix(1, 0));
    roll = atan2(matrix(2, 1), matrix(2, 2));
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

std::string extract_trajectory_strings(const std::string& file_path)
{
    std::string result = "";
    std::ifstream file(file_path);

    // 检查文件是否成功打开
    if (!file.is_open())
    {
        std::cerr << "Unable to open file: " << file_path << std::endl;
        return result;
    }

    std::string line;
    std::string content;

    // 读取整个文件内容
    while (std::getline(file, line))
    {
        content += line + "\n";
    }

    // 查找 [$trajectory: 和 $] 之间的内容
    size_t start_pos = 0;
    while ((start_pos = content.find("[$trajectory:", start_pos)) != std::string::npos)
    {
        size_t end_pos = content.find("$]", start_pos);
        if (end_pos != std::string::npos)
        {
            // 提取 [$trajectory: 和 $] 之间的内容
            start_pos += 13;  // 跳过 "[$trajectory:"
            std::string trajectory = content.substr(start_pos, end_pos - start_pos);
            result = trajectory;
            break;
        }
    }

    return result;
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
