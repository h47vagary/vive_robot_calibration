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

void CalibrationManager::set_tool_calibration_pose_flange(const int &index, const CartesianPose &pose)
{
    if (index >= flange_poses_.size())
    {
        flange_poses_.resize(index + 1);
    }
    
    flange_poses_[index] = pose;
}

void CalibrationManager::set_tool_calibration_pose_tcp(const int &index, const CartesianPose &pose)
{
    if (index >= tcp_poses_.size())
    {
        tcp_poses_.resize(index + 1);
    }
    
    tcp_poses_[index] = pose;
}

void CalibrationManager::get_tool_calibration_pose(std::vector<CartesianPose> &flange_poses, std::vector<CartesianPose> &tcp_poses)
{
    flange_poses = flange_poses_;
    tcp_poses = tcp_poses_;
}

void CalibrationManager::get_tool_calibrated(bool &calibrated)
{
    calibrated = tool_calibrated_;
}

void CalibrationManager::get_tool_max_error(double &max_error)
{
    max_error = tool_max_error_;
}

void CalibrationManager::get_calibration_matrix(Eigen::Matrix4d &pose_calibration_matrix)
{
    pose_calibration_matrix = *tool_pose_calibration_matrix_;
}

int CalibrationManager::clear_tool_calibration_pose()
{
    flange_poses_.clear();
    tcp_poses_.clear();
    tool_calibrated_ = false;
    tool_max_error_ = -1000;
    return 0;
}

Eigen::Matrix4d CalibrationManager::pose_to_matrix(const CartesianPose &pose)
{
    Eigen::AngleAxisd rz(pose.orientation.C, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd ry(pose.orientation.B, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rx(pose.orientation.A, Eigen::Vector3d::UnitX());

    Eigen::Matrix3d R = (rz * ry * rx).toRotationMatrix();
    Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    return T;
}

int CalibrationManager::calculate_toolhand_calibration_matrix(const std::vector<CartesianPose> &flange_poses, const std::vector<CartesianPose> &tcp_poses)
{
    if (flange_poses.size() != 6 || tcp_poses.size() != 6)
    {
        std::cout << "需要的标定点数为 6 个" << std::endl;
    }

    std::vector<Eigen::Matrix4d> f_list, t_list;
    for (int i = 0; i < 6; ++i)
    {
        f_list.push_back(pose_to_matrix(flange_poses[i]));
        t_list.push_back(pose_to_matrix(tcp_poses[i]));
    }

    Eigen::MatrixXd flange_mat(3, 6), tcp_mat(3, 6);

    for (int i = 0; i < 6; ++i)
    {
        flange_mat.col(i)= f_list[i].block<3,1>(0,3);
        tcp_mat.col(i) = t_list[i].block<3,1>(0,3);
    }

    Eigen::Vector3d flang_center = flange_mat.rowwise().mean();
    Eigen::Vector3d tcp_center = tcp_mat.rowwise().mean();

    flange_mat.colwise() -= flang_center;
    tcp_mat.colwise() -= tcp_center;

    Eigen::Matrix3d H = flange_mat * tcp_mat.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0)
    {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }

    Eigen::Vector3d t = tcp_center - R * flang_center;
    (*tool_pose_calibration_matrix_) = Eigen::Matrix4d::Identity();
    (*tool_pose_calibration_matrix_).block<3,3>(0,0) = R;
    (*tool_pose_calibration_matrix_).block<3,1>(0,3) = t;

    return 0;
}