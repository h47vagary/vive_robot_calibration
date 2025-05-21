/**
 * @file calibration.h
 * @brief 标定算法器
 * @version 0.1
 * @date 2025-05-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include "utils.h"

class CalibrationManager
{
public:
    CalibrationManager();
    ~CalibrationManager();

    int calibrate(double rz, double &error_out);
    void init(const bool &calibrated, const double &max_error,
              const Eigen::Matrix4d &position_calibration_matrix,
              const Eigen::Matrix3d &orientation_offset_matrix);
    void get_calibrated(bool &calibrated);
    void get_max_error(double &max_error);
    void get_position_calibration_matrix(Eigen::Matrix4d &position_calibration_matrix);
    void get_orientation_offset_matrix(Eigen::Matrix3d &orientation_offset_matrix);
    void get_calibration_positions(std::vector<CartesianPosition> &robot_calibration_positions,
                                   std::vector<CartesianPosition> &device_calibration_positions);
    void get_calibration_orientation(CartesianOrientation &robot_calibration_orientation,
                                     CartesianOrientation &device_calibration_orientation);

    void device_pose_to_robot_pose(const CartesianPosition &device_position, const CartesianOrientation &device_orientation, CartesianPose &robot_pose);
    void set_calibration_position(const int &index, const CartesianPosition &robot_position, const CartesianPosition &device_position);
    void set_calibration_orientation(const CartesianOrientation &robot_orientation, const CartesianOrientation &device_orientation);
    void set_robot_calibration_positon(const int &index, const CartesianPosition &robot_position);
    void set_device_calibration_position(const int &index, const CartesianPosition &device_position);
    void set_robot_calibration_orientation(const int &index, const CartesianOrientation &robot_orientation);
    void set_device_calibration_orientation(const int &index, const CartesianOrientation &device_orientation);
    
    int clear_calibration_position();
    void set_calibration_algorithm(int method);
    void get_calibration_algorithm(int& method);

private:
    int calculate_position_calibration_matrix(double &error_out);
    int calculate_position_calibration_matrix(double rz, double &error_out);
    int calculate_position_calibration_matrix_svd(double &error_out);
    int calculate_orientation_offset_matrix();

    void cartesian_orientation_to_matrix(const CartesianOrientation &orientation, Eigen::Matrix3d &matrix);
    void matrix_to_cartesian_orientation(const Eigen::Matrix3d &matrix, CartesianOrientation &orientation);

    void optimize_svd(const std::vector<Eigen::Vector3d> &robot_positions, const std::vector<Eigen::Vector3d> &device_positions,
                      Eigen::Matrix3d &rotation, Eigen::Vector3d &transformation);

    bool calibrated_;
    double max_error_;
    Eigen::Matrix4d *position_calibration_matrix_;
    Eigen::Matrix3d *orientation_offset_matrix_;

    std::vector<CartesianPosition> robot_calibration_positions_;
    std::vector<CartesianPosition> device_calibration_positions_;
    CartesianOrientation robot_calibration_orientation_;
    CartesianOrientation device_calibration_orientation_;

    int c_calibration_method = 2;    // 0-基于RZ值和位置的标定 1-九点标定 2-SVD标定
    const int c_max_iteration = 50;
};

/**
 * @brief 通用版本的 RobotModel::toolCalibration 接口抽出来的关于 6 点的工具手标定算法法
 * 
 */
class ToolCalibration6Points
{
public:
    ToolCalibration6Points();
    ~ToolCalibration6Points();

    int calibrate();

    void get_calibrated(bool &calibrated);
    void get_pose_calibration_matrix(Eigen::Matrix4d &pose_calibration_matrix);
    void get_calibration_poses(std::vector<CartesianPose> &calibration_poses);

    void set_calibration_pose(const int &index, const CartesianPose &pose);
    int clear_calibration_pose();

private:
    int tool_calculate_6points(const std::vector<CartesianPose> &poses, Eigen::Matrix4d& calib_matrix);
    Eigen::Matrix4d rpy_to_matrix(const CartesianPose& pose_deg);
    

private:
    std::vector<CartesianPose> source_poses_;
    Eigen::Matrix4d *calibration_matrix_;
    bool calibrated_;
};

// /**
//  * @brief 最小二乘法标定工具手
//  * 
//  */
// class ToolCalibrationLeastSquare
// {
// public:
//     ToolCalibrationLeastSquare();
//     ~ToolCalibrationLeastSquare();

//     int calibrate();
//     void get_calibrated(bool &calibrated) const;
//     void get_tcp_in_flange(Eigen::Vector3d &tcp_pos) const;

//     void get_calibration_poses(std::vector<CartesianPose> &calibration_poses) const;
//     void set_calibration_pose(const int &index, const CartesianPose &pose);
//     int clear_calibration_pose();

// private:
//     int slove_least_square(const std::vector<CartesianPose> &poses, Eigen::Vector3d &tcp);

// private:
//     std::vector<CartesianPose> source_poses_;
//     Eigen::Vector3d *tcp_pos_;   // 法兰坐标系下的TCP位置
//     bool calibrated_;
// };

/**
 * @brief 通用版本的 RobotModel::toolCalibration 接口抽出来的关于 7 点的工具手标定算法法
 * 
 */
class ToolCalibration7Points
{
public:
    ToolCalibration7Points();
    ~ToolCalibration7Points();

    int calibrate();

    void get_calibrated(bool &calibrated);
    void get_pose_calibration_matrix(Eigen::Matrix4d &pose_calibration_matrix);
    void get_calibration_poses(std::vector<CartesianPose> &calibration_poses);
    void get_calibration_pos_vec(Eigen::Vector3d &pos_calibration_vec);

    void set_calibration_pose(const int &index, const CartesianPose &pose);
    int clear_calibration_pose();

private:
    int tool_calculate_7points(const std::vector<CartesianPose> &poses, Eigen::Matrix4d& calib_matrix, Eigen::Vector3d& pos_vec);
    int Sphere(double x1, double y1, double z1, double x2, double y2, double z2,
        double x3, double y3, double z3, double x4, double y4, double z4,
        double *px, double *py, double *pz, Eigen::Vector3d& vec);

private:
    std::vector<CartesianPose> source_poses_;
    Eigen::Matrix4d *calibration_matrix_;
    Eigen::Vector3d *calibration_pos_vec_;
    bool calibrated_;
};
#endif
