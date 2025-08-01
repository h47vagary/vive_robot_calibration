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
        std::cout << "use calibration based on Rz value and position" << std::endl;
        result = calculate_position_calibration_matrix(rz, error_out);
    }
    else if(c_calibration_method == 1)
    {
        std::cout << "use nine-point calibration" << std::endl;
        result = calculate_position_calibration_matrix(error_out);
    }
    else if(c_calibration_method == 2)
    {
        std::cout << "use svd calibration" << std::endl;
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

    std::cout << std::endl;
    std::cout << "===============device_pose_to_robot_pose begin=======================" << std::endl;
    std::cout << "device_orientation: " << device_orientation.A << device_orientation.B << device_orientation.C << std::endl;
    std::cout << "device_orientation_matrix: " << std::endl << device_orientation_matrix << std::endl;
    std::cout << "*orientation_offset_matrix_: " << std::endl << *orientation_offset_matrix_ << std::endl;
    std::cout << "robot_orientation_matrix: " << std::endl << robot_orientation_matrix << std::endl;
    std::cout << "===============device_pose_to_robot_pose end=======================" << std::endl;
    matrix_to_cartesian_orientation(robot_orientation_matrix, robot_pose.orientation);
}

void CalibrationManager::set_calibration_matrix(const Eigen::Matrix4d &position_calibration_matrix, const Eigen::Matrix3d &orientation_offset_matrix)
{
    *position_calibration_matrix_ = position_calibration_matrix;
    *orientation_offset_matrix_ = orientation_offset_matrix;
}

void CalibrationManager::set_calibration_position_matrix(const Eigen::Matrix4d &position_calibration_matrix)
{
    if(position_calibration_matrix.rows() != 4 || position_calibration_matrix.cols() != 4)
    {
        std::cerr << "Invalid position calibration matrix size." << std::endl;
        return;
    }
    *position_calibration_matrix_ = position_calibration_matrix;
}

void CalibrationManager::set_calibration_orientation_offset_matrix(const Eigen::Matrix3d &orientation_offset_matrix)
{
    if(orientation_offset_matrix.rows() != 3 || orientation_offset_matrix.cols() != 3)
    {
        std::cerr << "Invalid orientation offset matrix size." << std::endl;
        return;
    }
    *orientation_offset_matrix_ = orientation_offset_matrix;
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
    if(n < 2 ||  device_calibration_positions_.size() != n)
    {
        std::cerr << "Not enough calibration points or mismatched sizes." << std::endl;
        std::cerr << "robot_calibration_positions_.size() = " << robot_calibration_positions_.size() << std::endl;
        std::cerr << "device_calibration_positions_.size() = " << device_calibration_positions_.size() << std::endl;
        max_error_ = error_out = -1000;
        return -1;
    }
    Eigen::Vector3d robot_position_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d device_position_centroid = Eigen::Vector3d::Zero();
    for(int i = 0; i < n; ++i)
    {
        robot_position_centroid(0) += robot_calibration_positions_[i].x;
        robot_position_centroid(1) += robot_calibration_positions_[i].y;
        robot_position_centroid(2) += robot_calibration_positions_[i].z;
        device_position_centroid(0) += device_calibration_positions_[i].x;
        device_position_centroid(1) += device_calibration_positions_[i].y;
        device_position_centroid(2) += device_calibration_positions_[i].z;
    }
    robot_position_centroid /= n;
    device_position_centroid /= n;
    Eigen::MatrixXd P(3, n), Q(3, n);
    for(int i = 0; i < n; ++i)
    {
        P(0, i) = device_calibration_positions_[i].x - device_position_centroid(0);
        P(1, i) = device_calibration_positions_[i].y - device_position_centroid(1);
        P(2, i) = device_calibration_positions_[i].z - device_position_centroid(2);
        Q(0, i) = robot_calibration_positions_[i].x - robot_position_centroid(0);
        Q(1, i) = robot_calibration_positions_[i].y - robot_position_centroid(1);
        Q(2, i) = robot_calibration_positions_[i].z - robot_position_centroid(2);
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
    (*position_calibration_matrix_).block<3, 3>(0, 0) = rotation;
    (*position_calibration_matrix_).col(3).head(3) = transformation;
    std::cout << "## position_calibration_matrix: " << std::endl;
    std::cout << *position_calibration_matrix_ << std::endl;
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

int CalibrationManager::calculate_orientation_offset_matrix()
{
    std::cout << __FUNCTION__ << std::endl;
    Eigen::Matrix3d robot_orientation_matrix;
    Eigen::Matrix3d device_orientation_matrix;
    cartesian_orientation_to_matrix(robot_calibration_orientation_, robot_orientation_matrix);
    cartesian_orientation_to_matrix(device_calibration_orientation_, device_orientation_matrix);

    std::cout << "robot_orientation_matrix: " << std::endl;
    std::cout << robot_orientation_matrix << std::endl;

    std::cout << "device_orientation_matrix: " << std::endl;
    std::cout << device_orientation_matrix << std::endl;

    if(fabs(device_orientation_matrix.determinant()) < EPSILON) return -1;
    // *orientation_offset_matrix_ = robot_orientation_matrix * device_orientation_matrix.transpose();       // 补偿方向：设备坐标变换到robot坐标
    *orientation_offset_matrix_ = device_orientation_matrix.transpose() * robot_orientation_matrix;          // 补偿方向：robot坐标变换到设备坐标
    std::cout << __FUNCTION__ << " *orientation_offset_matrix_:" << std::endl;
    std::cout << *orientation_offset_matrix_ << std::endl;
    double A, B, C;
    Utils::matrix_to_eular_ABC(*orientation_offset_matrix_, A, B, C);
    std::cout << "*orientation_offset_matrix_->A " << A << " B " << B << " C " << C << std::endl;

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


//========================================= 七点标定 ============================================
ToolCalibration7Points::ToolCalibration7Points()
{
    calibration_matrix_ = new Eigen::Matrix4d();
    calibration_pos_vec_ = new Eigen::Vector4d();
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

void ToolCalibration7Points::init(const bool &calibrated, const Eigen::Matrix4d &position_calibration_matrix, const Eigen::Vector4d &calibration_pos_vec)
{
    calibrated_ = calibrated;
    *calibration_matrix_ = position_calibration_matrix;
    *calibration_pos_vec_ = calibration_pos_vec;
}

void ToolCalibration7Points::get_calibrated(bool &calibrated)
{
    calibrated = calibrated_;
}

void ToolCalibration7Points::get_ori_calibration_matrix(Eigen::Matrix4d &ori_calibration_matrix)
{
    ori_calibration_matrix = *calibration_matrix_;
}

void ToolCalibration7Points::get_calibration_poses(std::vector<CartesianPose> &calibration_poses)
{
    calibration_poses = source_poses_;
}

void ToolCalibration7Points::get_calibration_pos_vec(Eigen::Vector4d &pos_calibration_vec)
{
    pos_calibration_vec = *calibration_pos_vec_;
}

void ToolCalibration7Points::set_ori_calibration_matrix(const Eigen::Matrix4d &ori_calibration_matrix)
{
    *calibration_matrix_ = ori_calibration_matrix;
}

void ToolCalibration7Points::set_calibration_pos_vec(const Eigen::Vector4d &calibration_pos_vec)
{
    *calibration_pos_vec_ = calibration_pos_vec;
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

int ToolCalibration7Points::tool_calculate_7points(const std::vector<CartesianPose> &poses, Eigen::Matrix4d& calib_matrix, Eigen::Vector4d& pos_vec)
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
           &ox, &oy, &oz);
    vecSphereCentre(0) = ox;
    vecSphereCentre(1) = oy;
    vecSphereCentre(2) = oz;
    vecSphereCentre(3) = 1.0;
    std::cout << __FUNCTION__ << " ox: " << ox << " oy: " << oy << " oz: " << oz << std::endl;

    matPos1 = Utils::pose_to_matrix(poses[0]);
    vecTool2Flange = matPos1.inverse()*vecSphereCentre;     //P(Px Py Pz) in the tool coordinate system
    pos_vec = vecTool2Flange;

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

    return 0;
}
int ToolCalibration7Points::Sphere(double x1, double y1, double z1, double x2, double y2, double z2,
        double x3, double y3, double z3, double x4, double y4, double z4,
        double *px, double *py, double *pz)
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

    std::cout << __FUNCTION__ << std::endl;
    std::cout << "x1: " << x1 << " *px: " << *px << " x1-*px: " << x1-*px << "\n"
              << "y1: " << y1 << " *py: " << *py << " y1-*py: " << y1-*py << "\n"
              << "z1: " << z1 << " *pz: " << *pz << " z1-*pz: " << z1-*pz << std::endl;
                
    return 0;
}