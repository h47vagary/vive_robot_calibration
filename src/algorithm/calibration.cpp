#include "calibration.h"
#include <iostream>

CalibrationManager::CalibrationManager()
{
    position_calibration_matrix_ = new Eigen::Matrix4d();
    orientation_offset_matrix_ = new Eigen::Matrix3d();
    init(false, 0, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity());
}

CalibrationManager::~CalibrationManager()
{
    delete position_calibration_matrix_;
    delete orientation_offset_matrix_;
}

int CalibrationManager::calibrate(double rz, double &error_out)
{
    init(false, 0, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity());
    int result;
    if(c_calibration_method == 0)
    {
        std::cout << "使用基于Rz值和位置的标定" << std::endl;
        result = calculate_position_calibration_matrix(rz, error_out);
    }
    else if(c_calibration_method == 1)
    {
        std::cout << "使用九点标定" << std::endl;
        result = calculate_position_calibration_matrix(error_out);
    }
    else if(c_calibration_method == 2)
    {
        std::cout << "使用SVD标定" << std::endl;
        result = calculate_position_calibration_matrix_svd(error_out);
    }
    else
    {
        return -1;
    }
    if(result != 0)
    {
        max_error_=error_out=-1000;
        return result;
    }
    result = calculate_orientation_offset_matrix();
    if(result != 0)
    {
        max_error_=error_out=-1000;
        return result;
    }
    calibrated_ = true;
    max_error_=error_out;
    return 0;
}

void CalibrationManager::init(const bool &calibrated, const double &max_error,
                              const Eigen::Matrix4d &position_calibration_matrix,
                              const Eigen::Matrix3d &orientation_offset_matrix)
{
    calibrated_ = calibrated;
    max_error_ = max_error;
    *position_calibration_matrix_ = position_calibration_matrix;
    *orientation_offset_matrix_ = orientation_offset_matrix;
}

void CalibrationManager::get_calibrated(bool &calibrated)
{
    calibrated = calibrated_;
}

void CalibrationManager::get_max_error(double &max_error)
{
    max_error = max_error_;
}

void CalibrationManager::get_position_calibration_matrix(Eigen::Matrix4d &position_calibration_matrix)
{
    position_calibration_matrix = *position_calibration_matrix_;
}

void CalibrationManager::get_orientation_offset_matrix(Eigen::Matrix3d &orientation_offset_matrix)
{
    orientation_offset_matrix = *orientation_offset_matrix_;
}

void CalibrationManager::get_calibratoin_matrix(Eigen::Matrix4d &pose_calibration_matrix)
{
    pose_calibration_matrix = *position_calibration_matrix_;
    pose_calibration_matrix.block<3, 3>(0, 0) = *orientation_offset_matrix_;
}

void CalibrationManager::get_calibration_positions(std::vector<CartesianPosition> &robot_calibration_positions,
                                                   std::vector<CartesianPosition> &device_calibration_positions)
{
    robot_calibration_positions = robot_calibration_positions_;
    device_calibration_positions = device_calibration_positions_;
}

void CalibrationManager::get_calibration_orientation(CartesianOrientation &robot_calibration_orientation,
                                                     CartesianOrientation &device_calibration_orientation)
{
    robot_calibration_orientation = robot_calibration_orientation_;
    device_calibration_orientation = device_calibration_orientation_;
}

void CalibrationManager::device_pose_to_robot_pose(const CartesianPosition &device_position,
                                                   const CartesianOrientation &device_orientation,
                                                   CartesianPose &robot_pose)
{
    Eigen::Vector4d device_position_vector, robot_position_vector;
    device_position_vector << device_position.x, device_position.y, device_position.z, 1;
    robot_position_vector = *position_calibration_matrix_ * device_position_vector;
    robot_pose.position.x = robot_position_vector(0);
    robot_pose.position.y = robot_position_vector(1);
    robot_pose.position.z = robot_position_vector(2);

    Eigen::Matrix3d device_orientation_matrix, robot_orientation_matrix;
    cartesian_orientation_to_matrix(device_orientation, device_orientation_matrix);
    robot_orientation_matrix = *orientation_offset_matrix_ * device_orientation_matrix;
    matrix_to_cartesian_orientation(robot_orientation_matrix, robot_pose.orientation);
}

void CalibrationManager::set_calibration_position(const int &index, const CartesianPosition &robot_position, const CartesianPosition &device_position)
{
    if(index >= robot_calibration_positions_.size())
    {
        robot_calibration_positions_.resize(index + 1);
    }
    if(index >= device_calibration_positions_.size())
    {
        device_calibration_positions_.resize(index + 1);
    }
    robot_calibration_positions_[index] = robot_position;
    device_calibration_positions_[index] = device_position;
}

void CalibrationManager::set_calibration_orientation(const CartesianOrientation &robot_orientation, const CartesianOrientation &device_orientation)
{
    robot_calibration_orientation_ = robot_orientation;
    device_calibration_orientation_ = device_orientation;
}

void CalibrationManager::set_robot_calibration_positon(const int &index, const CartesianPosition &robot_position)
{
    if(index >= robot_calibration_positions_.size())
    {
        robot_calibration_positions_.resize(index + 1);
    }
    robot_calibration_positions_[index] = robot_position;
}

void CalibrationManager::set_device_calibration_position(const int &index, const CartesianPosition &device_position)
{
    if(index >= device_calibration_positions_.size())
    {
        device_calibration_positions_.resize(index + 1);
    }
    device_calibration_positions_[index] = device_position;
}

void CalibrationManager::set_robot_calibration_orientation(const int &index, const CartesianOrientation &robot_orientation)
{
    robot_calibration_orientation_ = robot_orientation;
}

void CalibrationManager::set_device_calibration_orientation(const int &index, const CartesianOrientation &device_orientation)
{
    device_calibration_orientation_ = device_orientation;
}

int CalibrationManager::clear_calibration_position()
{
    robot_calibration_positions_.clear();
    device_calibration_positions_.clear();
    calibrated_=false;
    max_error_=-1000;

    return 1;
}

void CalibrationManager::set_calibration_algorithm(int method)
{
    c_calibration_method = method;
}

void CalibrationManager::get_calibration_algorithm(int &method)
{
    method = c_calibration_method;
}

int CalibrationManager::calculate_position_calibration_matrix(double &error_out)
{
    int n = robot_calibration_positions_.size();
    if(n < 4 ||  device_calibration_positions_.size() != n) return -1;

    Eigen::MatrixXd A(n, 4);
    for(int j = 0; j < n; j++)
    {
        A(j, 0) = device_calibration_positions_[j].x;
        A(j, 1) = device_calibration_positions_[j].y;
        A(j, 2) = device_calibration_positions_[j].z;
        A(j, 3) = 1;
    }
    Eigen::MatrixXd A_tmp(4, 4);
    A_tmp = A.transpose() * A;
    if(fabs(A_tmp.determinant()) < EPSILON) return -1;

    Eigen::VectorXd b(n);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < n; j++)
        {
            switch (i) {
            case 0:
                b(j) = robot_calibration_positions_[j].x;
                break;
            case 1:
                b(j) = robot_calibration_positions_[j].y;
                break;
            case 2:
                b(j) = robot_calibration_positions_[j].z;
                break;
            }
        }
        Eigen::VectorXd x(3);
        x = A_tmp.inverse() * A.transpose() * b;
        (*position_calibration_matrix_)(i, 0) = x(0);
        (*position_calibration_matrix_)(i, 1) = x(1);
        (*position_calibration_matrix_)(i, 2) = x(2);
        (*position_calibration_matrix_)(i, 3) = x(3);
        error_out = std::max(max_error_, (A * x - b).norm());
        std::cout<<"calculate_position_calibration_matrix "<<i<<" "<< x(0)<<" "<< x(1)<<" "<< x(2)<<" "<< x(3)<<std::endl;
    }
    return 0;
}

int CalibrationManager::calculate_position_calibration_matrix(double rz, double &error_out)
{
    int n = robot_calibration_positions_.size();
    if(n < 1 ||  device_calibration_positions_.size() != n) return -1;

    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(PI + rz, Eigen::Vector3d::UnitZ());
    (*position_calibration_matrix_).block<3, 3>(0, 0) = rotation;

    double tx_sum = 0, ty_sum = 0, tz_sum = 0;
    for(int i = 0; i < n; ++i)
    {
        tx_sum += robot_calibration_positions_[i].x - (rotation(0, 0) * device_calibration_positions_[i].x + rotation(0, 1) * device_calibration_positions_[i].y + rotation(0, 2) * device_calibration_positions_[i].z);
        ty_sum += robot_calibration_positions_[i].y - (rotation(1, 0) * device_calibration_positions_[i].x + rotation(1, 1) * device_calibration_positions_[i].y + rotation(1, 2) * device_calibration_positions_[i].z);
        tz_sum += robot_calibration_positions_[i].z - (rotation(2, 0) * device_calibration_positions_[i].x + rotation(2, 1) * device_calibration_positions_[i].y + rotation(2, 2) * device_calibration_positions_[i].z);
    }
    Eigen::Vector3d transformation;
    transformation << tx_sum / n, ty_sum / n, tz_sum / n;
    (*position_calibration_matrix_).col(3).head(3) = transformation;

    for(int i = 0; i < n; ++i)
    {
        Eigen::VectorXd d(3);
        d << device_calibration_positions_[i].x, device_calibration_positions_[i].y, device_calibration_positions_[i].z;
        Eigen::VectorXd r(3);
        r << robot_calibration_positions_[i].x, robot_calibration_positions_[i].y, robot_calibration_positions_[i].z;
        Eigen::Vector3d cr = rotation * d + transformation - r;
        double error = sqrt(pow(cr(1), 2) + pow(cr(2), 2));
        max_error_ = std::max(max_error_, error);
        std::cout<<"error : "<<error<<std::endl;
    }
    error_out = max_error_;
    return 0;
}

int CalibrationManager::calculate_position_calibration_matrix_svd(double &error_out)
{
    int n = robot_calibration_positions_.size();
    if(n < 2 ||  device_calibration_positions_.size() != n) return -1;

    std::vector<Eigen::Vector3d> robot_positions, device_positions;
    Eigen::Vector3d robot_position_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d device_position_centroid = Eigen::Vector3d::Zero();
    for(int i = 0; i < n; ++i)
    {
        Eigen::Vector3d robot_position;
        robot_position(0) = robot_calibration_positions_[i].x;
        robot_position(1) = robot_calibration_positions_[i].y;
        robot_position(2) = robot_calibration_positions_[i].z;
        robot_positions.push_back(robot_position);
        robot_position_centroid += robot_position;

        Eigen::Vector3d device_position;
        device_position(0) = device_calibration_positions_[i].x;
        device_position(1) = device_calibration_positions_[i].y;
        device_position(2) = device_calibration_positions_[i].z;
        device_positions.push_back(device_position);
        device_position_centroid += device_position;
    }
    robot_position_centroid /= n;
    device_position_centroid /= n;

    Eigen::MatrixXd P(3, n), Q(3, n);
    for(int i = 0; i < n; ++i)
    {
        P.col(i) = device_positions[i] - device_position_centroid;
        Q.col(i) = robot_positions[i] - robot_position_centroid;
    }

    Eigen::Matrix3d H = P * Q.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d rotation = V * U.transpose();
    if(rotation.determinant() < EPSILON)
    {
        V.col(2) *= -1;
        rotation = V * U.transpose();
    }
    Eigen::Vector3d transformation = robot_position_centroid - rotation * device_position_centroid;
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            std::cout<<"333 rotation : "<<i<<", "<<j<<" : "<<rotation(i, j)<<std::endl;
        }
    }
    std::cout<<"333 transformation : "<<transformation(0)<<", "<<transformation(1)<<", "<<transformation(2)<<std::endl;
    optimize_svd(robot_positions, device_positions, rotation, transformation);
    (*position_calibration_matrix_).block<3, 3>(0, 0) = rotation;
    (*position_calibration_matrix_).col(3).head(3) = transformation;

    for(int i = 0; i < n; ++i)
    {
        Eigen::VectorXd d(3);
        d << device_calibration_positions_[i].x, device_calibration_positions_[i].y, device_calibration_positions_[i].z;
        Eigen::VectorXd r(3);
        r << robot_calibration_positions_[i].x, robot_calibration_positions_[i].y, robot_calibration_positions_[i].z;
        Eigen::Vector3d cr = rotation * d + transformation - r;
        double error = cr.norm();
        max_error_ = std::max(max_error_, error);
        std::cout<<"error : "<<error<<std::endl;
    }
    error_out = max_error_;
    return 0;
}

int CalibrationManager::calculate_orientation_offset_matrix()
{
    Eigen::Matrix3d robot_orientation_matrix;
    Eigen::Matrix3d device_orientation_matrix;
    cartesian_orientation_to_matrix(robot_calibration_orientation_, robot_orientation_matrix);
    cartesian_orientation_to_matrix(device_calibration_orientation_, device_orientation_matrix);

    if(fabs(device_orientation_matrix.determinant()) < EPSILON) return -1;
    *orientation_offset_matrix_ = robot_orientation_matrix * device_orientation_matrix.transpose();

    return 0;
}

void CalibrationManager::cartesian_orientation_to_matrix(const CartesianOrientation &orientation, Eigen::Matrix3d &matrix)
{
    Utils::euler_ABC_to_matrix(orientation.A, orientation.B, orientation.C, matrix);
}

void CalibrationManager::matrix_to_cartesian_orientation(const Eigen::Matrix3d &matrix, CartesianOrientation &orientation)
{
    Utils::matrix_to_eular_ABC(matrix, orientation.A, orientation.B, orientation.C);
}

void CalibrationManager::optimize_svd(const std::vector<Eigen::Vector3d> &robot_positions, const std::vector<Eigen::Vector3d> &device_positions, Eigen::Matrix3d &rotation, Eigen::Vector3d &transformation)
{
    Eigen::AngleAxisd init_rotation(rotation);
    Eigen::VectorXd params(6);
    params << init_rotation.angle() * init_rotation.axis(), transformation;

    for(int i = 0; i < c_max_iteration; ++i)
    {
        std::cout<<"444 i : "<<i<<std::endl;
        Eigen::MatrixXd jacobian(3 * robot_positions.size(), 6);
        Eigen::VectorXd residuals(3 * robot_positions.size());

        for(size_t i = 0; i < robot_positions.size(); ++i)
        {
            residuals.segment<3>(3 * i) = rotation * device_positions[i] + transformation - robot_positions[i];
            double theta = params.head<3>().norm();
            if(theta < EPSILON)
            {
                jacobian.block<3, 3>(3 * i, 0) << 0, device_positions[i](2), -device_positions[i](1),
                        -device_positions[i](2), 0, device_positions[i](0),
                        device_positions[i](1), -device_positions[i](0), 0;
            }
            else
            {
                Eigen::Matrix3d S;
                S << 0, -device_positions[i](2), device_positions[i](1),
                        device_positions[i](2), 0, -device_positions[i](0),
                        -device_positions[i](1), device_positions[i](0), 0;
                Eigen::Matrix3d jacobian_rotation = -rotation * S;
                jacobian.block<3, 3>(3 * i, 0) = jacobian_rotation;
            }
            jacobian.block<3, 3>(3 * i, 3) = Eigen::Matrix3d::Identity();
        }

        Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
        Eigen::MatrixXd H = jacobian_transpose * jacobian;
        Eigen::MatrixXd g = jacobian_transpose * residuals;
        Eigen::VectorXd delta = H.ldlt().solve(-g);
        for(int j = 0; j < delta.size(); ++j)
        {
            std::cout<<"333 delta : "<<j<<" : "<<delta(j)<<std::endl;
        }

        params += delta;
        if(delta.norm() < EPSILON)
        {
            break;
        }
    }

    Eigen::Vector3d theta = params.head<3>();
    if (theta.norm() < EPSILON)
    {
        rotation = Eigen::Matrix3d::Identity();
    }
    else
    {
        Eigen::AngleAxisd axis(theta.norm(), theta.normalized());
        rotation = axis.toRotationMatrix();
//        Eigen::Matrix3d K;
//        K << 0, -axis(2), axis(1),
//                axis(2), 0, -axis(0),
//                -axis(1), axis(0), 0;
//        rotation = Eigen::Matrix3d::Identity() + sin(theta) * K + (1 - cos(theta)) * K * K;
    }
    transformation = params.tail<3>();
}

ToolCalibration6Points::ToolCalibration6Points()
{
    calibration_matrix_ = new Eigen::Matrix4d();
    source_poses_.clear();
    calibrated_ = false;
}

ToolCalibration6Points::~ToolCalibration6Points()
{
    delete calibration_matrix_;
}

int ToolCalibration6Points::calibrate()
{
    if (source_poses_.size() != 6)
        return -1;

    if (!tool_calculate_6points(source_poses_, *calibration_matrix_))
    {
        calibrated_ = true;
        return 0;
    }
    else
    {
        calibrated_ = false;
        return -1;
    }
}

void ToolCalibration6Points::get_calibrated(bool &calibrated)
{
    calibrated = calibrated_;
}

void ToolCalibration6Points::get_pose_calibration_matrix(Eigen::Matrix4d &pose_calibration_matrix)
{
    pose_calibration_matrix = *calibration_matrix_;
}

void ToolCalibration6Points::get_calibration_poses(std::vector<CartesianPose> &calibration_poses)
{
    calibration_poses = source_poses_;
}

void ToolCalibration6Points::set_calibration_pose(const int &index, const CartesianPose &pose)
{
    if (index >= source_poses_.size())
    {
        source_poses_.resize(index+1);
    }
    source_poses_[index] = pose;
}

int ToolCalibration6Points::clear_calibration_pose()
{
    source_poses_.clear();
    calibrated_ = false;
    return 0;
}

int ToolCalibration6Points::tool_calculate_6points(const std::vector<CartesianPose> &poses, Eigen::Matrix4d &calib_matrix)
{
    if (poses.size() != 6)
        return -1;

    // Step 1: XY 平面标定（平移量）
    // Eigen::Matrix4d T1 = rpy_to_matrix(poses[0]);
    // Eigen::Matrix4d T2 = rpy_to_matrix(poses[1]);
    Eigen::Matrix4d T1 = Utils::pose_to_matrix(poses[0]);
    Eigen::Matrix4d T2 = Utils::pose_to_matrix(poses[1]);

    double a = T1(0, 0) - T2(0, 0);
    double b = T1(0, 1) - T2(0, 1);
    double c = T1(1, 0) - T2(1, 0);
    double d = T1(1, 1) - T2(1, 1);
    double e = T2(0, 3) - T1(0, 3);
    double f = T2(1, 3) - T1(1, 3);

    Eigen::Vector3d offset;
    offset[0] = (e * d - b * f) / (a * d - b * c);
    offset[1] = (e * c - a * f) / (b * c - a * d);
    offset[2] = 0.0;

    // Step 2: Z 值
    //Eigen::Matrix4d T3 = rpy_to_matrix(poses[2]);
    Eigen::Matrix4d T3 = Utils::pose_to_matrix(poses[2]);
    double dz = (T3(0, 3) - T1(0, 3) - offset[0] * (T1(0, 0) - T3(0, 0)) - offset[1] * (T1(0, 1) - T3(0, 1))) 
                / (T1(0, 2) - T3(0, 2));
    offset[2] = dz;

    // Step 3: 构造方向基（方向向量）
    Eigen::Vector3d vec1, vec2, vec3;
    vec1 << poses[4].position.x - poses[3].position.x,
            poses[4].position.y - poses[3].position.y,
            poses[4].position.z - poses[3].position.z;
    vec2 << poses[5].position.x - poses[3].position.x,
            poses[5].position.y - poses[3].position.y,
            poses[5].position.z - poses[3].position.z;

    vec3 = vec1.cross(vec2);
    vec2 = vec3.cross(vec1);

    vec1.normalize();
    vec2.normalize();
    vec3.normalize();

    // Step 4: 构造工具坐标系的齐次变换矩阵
    calib_matrix = Eigen::Matrix4d::Identity();
    calib_matrix.block<3, 1>(0, 0) = vec1;
    calib_matrix.block<3, 1>(0, 1) = vec2;
    calib_matrix.block<3, 1>(0, 2) = vec3;
    calib_matrix.block<3, 1>(0, 3) = offset;

    return 0;
}

// 欧拉角转旋转矩阵
Eigen::Matrix4d ToolCalibration6Points::rpy_to_matrix(const CartesianPose &pose_deg)
{
    double rx = pose_deg.orientation.A * PI / 180.0;
    double ry = pose_deg.orientation.B * PI / 180.0;
    double rz = pose_deg.orientation.C * PI / 180.0;

    double cr = cos(rx), sr = sin(rx);
    double cp = cos(ry), sp = sin(ry);
    double cy = cos(rz), sy = sin(rz);

    Eigen::Matrix3d R;
    R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
         sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
         -sp,     cp * sr,               cp * cr;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T(0, 3) = pose_deg.position.x;
    T(1, 3) = pose_deg.position.y;
    T(2, 3) = pose_deg.position.z;

    return T;
}



// ToolCalibrationLeastSquare::ToolCalibrationLeastSquare()
// {
//     source_poses_.clear();
//     calibrated_ = false;
//     tcp_pos_ = new Eigen::Vector3d();
// }

// ToolCalibrationLeastSquare::~ToolCalibrationLeastSquare()
// {
//     delete tcp_pos_;
// }

// int ToolCalibrationLeastSquare::calibrate()
// {
//     if (source_poses_.size() < 2)
//         return -1;

//     int result = slove_least_square(source_poses_, *tcp_pos_);
//     calibrated_ = (result == 0);
//     return result;
// }

// void ToolCalibrationLeastSquare::get_calibrated(bool &calibrated) const
// {
//     calibrated = calibrated_;
// }

// void ToolCalibrationLeastSquare::get_tcp_in_flange(Eigen::Vector3d &tcp_pos) const
// {
//     tcp_pos = *tcp_pos_;
// }

// void ToolCalibrationLeastSquare::get_calibration_poses(std::vector<CartesianPose> &calibration_poses) const
// {
//     calibration_poses = source_poses_;
// }

// void ToolCalibrationLeastSquare::set_calibration_pose(const int &index, const CartesianPose &pose)
// {
//     if (index > source_poses_.size())
//     {
//         source_poses_.resize(index + 1);
//     }
//     source_poses_[index] = pose;
// }

// int ToolCalibrationLeastSquare::clear_calibration_pose()
// {
//     source_poses_.clear();
//     calibrated_ = false;
//     tcp_pos_->setZero();
//     return 0;
// }

// int ToolCalibrationLeastSquare::slove_least_square(const std::vector<CartesianPose> &poses, Eigen::Vector3d &tcp)
// {
//     const size_t N = poses.size();
//     if (N < 2)
//         return -1;
    
//     std::vector<Eigen::Matrix4d> mats;
//     for (const auto& p : poses)
//     {
//         mats.push_back(Utils::pose_to_matrix(p));
//     }

//     std::vector<Eigen::Matrix3d> R_list;
//     std::vector<Eigen::Vector3d> T_list;

//     for (size_t i = 0; i < N - 1; ++i)
//     {
//         Eigen::Matrix4d T1 = mats[i];
//         Eigen::Matrix4d T2 = mats[i + 1];

//         Eigen::Matrix4d T_rel = T1.inverse() * T2;
//         Eigen::Matrix3d R = T_rel.block<3, 3>(0, 0);
//         Eigen::Vector3d t = T_rel.block<3, 1>(0, 3);

//         R_list.push_back(R - Eigen::Matrix3d::Identity());
//         T_list.push_back(t);
//     }

//     const size_t M = R_list.size();
//     Eigen::MatrixXd A(3 * M, 3);
//     Eigen::VectorXd B(3 * M);

//     for (size_t i = 0; i < M; ++i)
//     {
//         A.block<3,3>(3 * i, 0) = R_list[i];
//         B.segment<3>(3 * i) = T_list[i];
//     }

//     tcp = A.colPivHouseholderQr().solve(B);

//     return 0;
// }


//========================================= 七点标定 ============================================
ToolCalibration7Points::ToolCalibration7Points()
{
    calibration_matrix_ = new Eigen::Matrix4d();
    calibration_pos_vec_ = new Eigen::Vector3d();
    source_poses_.clear();
    calibrated_ = false;
}

ToolCalibration7Points::~ToolCalibration7Points()
{
    delete calibration_matrix_;
    delete calibration_pos_vec_;
}

int ToolCalibration7Points::calibrate()
{
    if (source_poses_.size() != 7)
        return -1;

    if (!tool_calculate_7points(source_poses_, *calibration_matrix_, *calibration_pos_vec_))
    {
        calibrated_ = true;
        return 0;
    }
    else
    {
        calibrated_ = false;
        return -1;
    }
}

void ToolCalibration7Points::get_calibrated(bool &calibrated)
{
    calibrated = calibrated_;
}

void ToolCalibration7Points::get_pose_calibration_matrix(Eigen::Matrix4d &pose_calibration_matrix)
{
    pose_calibration_matrix = *calibration_matrix_;
}

void ToolCalibration7Points::get_calibration_poses(std::vector<CartesianPose> &calibration_poses)
{
    calibration_poses = source_poses_;
}

void ToolCalibration7Points::get_calibration_pos_vec(Eigen::Vector3d &pos_calibration_vec)
{
    pos_calibration_vec = *calibration_pos_vec_;
}

void ToolCalibration7Points::set_calibration_pose(const int &index, const CartesianPose &pose)
{
    if (index >= source_poses_.size())
        source_poses_.resize(index + 1);

    source_poses_[index] = pose;
}

int ToolCalibration7Points::clear_calibration_pose()
{
    source_poses_.clear();
    calibrated_ = false;
    return 0;
}

int ToolCalibration7Points::tool_calculate_7points(const std::vector<CartesianPose> &poses, Eigen::Matrix4d& calib_matrix, Eigen::Vector3d& pos_vec)
{
    if (poses.size() != 7)
        return -1;

    //to store the centre of the sphere
    double ox = 0.0, oy = 0.0, oz = 0.0;
    //vars used in the step1
    Eigen::Vector4d vecSphereCentre(4), vecTool2Flange(4);
    Eigen::Matrix4d matPos1;
    Eigen::VectorXd tcp1(6);
    Eigen::VectorXd tcp5(6);
    Eigen::VectorXd tcpFlange(6);
    //vars used in the step2
    Eigen::Vector4d vecPos5ToBase(4);
    Eigen::Vector3d vecTemp1(3),vecTemp2(3),vecTemp3(3);
    Eigen::Matrix4d matPos5, matTool2Base, matTool2Flange;
    //Step1: 求解圆心,然后解出TCP点到法兰末端的位置P(Px Py Pz)，存放在tcpToFlange指针中
    Sphere(poses[0].position.x, poses[0].position.y, poses[0].position.z, poses[1].position.x, poses[1].position.y, poses[1].position.z,
           poses[2].position.x,poses[2].position.y,poses[2].position.z, poses[3].position.x, poses[3].position.y, poses[3].position.z,
           &ox, &oy, &oz, pos_vec);
    vecSphereCentre(0) = ox;
    vecSphereCentre(1) = oy;
    vecSphereCentre(2) = oz;
    vecSphereCentre(3) = 1.0;
    std::cout << __FUNCTION__ << " ox: " << ox << " oy: " << oy << " oz: " << oz << std::endl;

    // tcp1(0) = tcpPos1.x;
    // tcp1(1) = tcpPos1.y;
    // tcp1(2) = tcpPos1.z;
    // tcp1(3) = deg2rad(tcpPos1.A);
    // tcp1(4) = deg2rad(tcpPos1.B);
    // tcp1(5) = deg2rad(tcpPos1.C);
    // matPos1 = rpy2tr(tcp1);
    matPos1 = Utils::pose_to_matrix(poses[0]);
    vecTool2Flange = matPos1.inverse()*vecSphereCentre;     //P(Px Py Pz) in the tool coordinate system

    // Step2: 再求相对于法兰盘末端的旋转方向
    // 1.求TCP点在基坐标系下的姿态（向量）
    vecTemp1(0) = poses[5].position.x - poses[4].position.x;
    vecTemp1(1) = poses[5].position.y - poses[4].position.y;
    vecTemp1(2) = poses[5].position.z - poses[4].position.z;
    vecTemp2(0) = poses[6].position.x - poses[4].position.x;
    vecTemp2(1) = poses[6].position.y - poses[4].position.y;
    vecTemp2(2) = poses[6].position.z - poses[4].position.z;
    vecTemp3 = vecTemp1.cross(vecTemp2);    // vector3=vector1叉乘vector2
    vecTemp2 = vecTemp3.cross(vecTemp1);    // vector2=vector3叉乘vector1
    //2.姿态（向量）单位化得到N、O、A矢量，即旋转矩阵R
    vecTemp1.normalize();
    vecTemp2.normalize();
    vecTemp3.normalize();
    //3.在Pos5处创建TCP点在基坐标系下的齐次变换矩阵bTt
    // tcp5(0) = tcpPos5.x;
    // tcp5(1) = tcpPos5.y;
    // tcp5(2) = tcpPos5.z;
    // tcp5(3) = deg2rad(tcpPos5.A);
    // tcp5(4) = deg2rad(tcpPos5.B);
    // tcp5(5) = deg2rad(tcpPos5.C);
    // matPos5 = rpy2tr(tcp5);
    matPos5 = Utils::pose_to_matrix(poses[4]);
    vecPos5ToBase = matPos5*vecTool2Flange; //position5 P(Px Py Pz)in the base coordinate system
    matTool2Base(0,0) = vecTemp1(0);
    matTool2Base(1,0) = vecTemp1(1);
    matTool2Base(2,0) = vecTemp1(2);
    matTool2Base(3,0) = 0.0;
    matTool2Base(0,1) = vecTemp2(0);
    matTool2Base(1,1) = vecTemp2(1);
    matTool2Base(2,1) = vecTemp2(2);
    matTool2Base(3,1) = 0.0;
    matTool2Base(0,2) = vecTemp3(0);
    matTool2Base(1,2) = vecTemp3(1);
    matTool2Base(2,2) = vecTemp3(2);
    matTool2Base(3,2) = 0.0;
    matTool2Base(0,3) = vecPos5ToBase(0);
    matTool2Base(1,3) = vecPos5ToBase(1);
    matTool2Base(2,3) = vecPos5ToBase(2);
    matTool2Base(3,3) = 1.0;
    matTool2Flange = matPos5.inverse()*matTool2Base;

    calib_matrix = matTool2Flange;

    // tcpFlange = tr2MCS(matTool2Flange);
    // for (AXIS_REFS_INDEX ai=0; ai<6; ++ai)
    // {
    //     if (fabs(tcpFlange[ai]) < 10e-4)
    //     {
    //         tcpFlange[ai] = 0;
    //     }
    // }
    // tcpToFlange->x = tcpFlange(0);
    // tcpToFlange->y = tcpFlange(1);
    // tcpToFlange->z = tcpFlange(2);
    // tcpToFlange->A = tcpFlange(3);
    // tcpToFlange->B = tcpFlange(4);
    // tcpToFlange->C = tcpFlange(5);
    return 0;
}
int ToolCalibration7Points::Sphere(double x1, double y1, double z1, double x2, double y2, double z2,
        double x3, double y3, double z3, double x4, double y4, double z4,
        double *px, double *py, double *pz, Eigen::Vector3d& vec)
{
    double a11, a12, a13, a21, a22, a23, a31, a32, a33, b1, b2, b3, d, d1, d2,
            d3;
    a11 = 2 * (x2 - x1);
    a12 = 2 * (y2 - y1);
    a13 = 2 * (z2 - z1);
    a21 = 2 * (x3 - x2);
    a22 = 2 * (y3 - y2);
    a23 = 2 * (z3 - z2);
    a31 = 2 * (x4 - x3);
    a32 = 2 * (y4 - y3);
    a33 = 2 * (z4 - z3);
    b1 = x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1 + z2 * z2 - z1 * z1;
    b2 = x3 * x3 - x2 * x2 + y3 * y3 - y2 * y2 + z3 * z3 - z2 * z2;
    b3 = x4 * x4 - x3 * x3 + y4 * y4 - y3 * y3 + z4 * z4 - z3 * z3;
    d = a11 * a22 * a33 + a12 * a23 * a31 + a13 * a21 * a32 - a11 * a23 * a32
            - a12 * a21 * a33 - a13 * a22 * a31;
    d1 = b1 * a22 * a33 + a12 * a23 * b3 + a13 * b2 * a32 - b1 * a23 * a32
            - a12 * b2 * a33 - a13 * a22 * b3;
    d2 = a11 * b2 * a33 + b1 * a23 * a31 + a13 * a21 * b3 - a11 * a23 * b3
            - b1 * a21 * a33 - a13 * b2 * a31;
    d3 = a11 * a22 * b3 + a12 * b2 * a31 + b1 * a21 * a32 - a11 * b2 * a32
            - a12 * a21 * b3 - b1 * a22 * a31;
    (*px) = d1 / d;
    (*py) = d2 / d;
    (*pz) = d3 / d;

    vec = Eigen::Vector3d(x1 -*px, y1 - *py, z1 - *pz);
    return 0;
}