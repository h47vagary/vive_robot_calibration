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


    //====== 工具手标定 ======//
public:
    void set_tool_calibration_pose_flange(const int &index, const CartesianPose &pose);
    void set_tool_calibration_pose_tcp(const int &index, const CartesianPose &pose);
    void get_tool_calibration_pose(std::vector<CartesianPose> &flange_poses, std::vector<CartesianPose> &tcp_poses);
    void get_tool_calibrated(bool &calibrated);
    void get_tool_max_error(double &max_error);
    void get_calibration_matrix(Eigen::Matrix4d &pose_calibration_matrix);
    int clear_tool_calibration_pose();
private:
    Eigen::Matrix4d pose_to_matrix(const CartesianPose& pose);
    int calculate_toolhand_calibration_matrix(const std::vector<CartesianPose>& flange_poses, 
                                                const std::vector<CartesianPose>& tcp_poses);
    double tool_max_error_;
    bool tool_calibrated_;
    std::vector<CartesianPose> flange_poses_;
    std::vector<CartesianPose> tcp_poses_;
    Eigen::Matrix4d *tool_pose_calibration_matrix_;
    
};

#endif
