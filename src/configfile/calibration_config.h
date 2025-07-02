/**
 * @file calibration_config.h
 * @brief 标定配置文件
 * @version 0.1
 * @date 2025-07-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once
#include <string>
#include "utils.h"
#include "json/json.h"


struct Calibration_robotbase_2_trackerbase
{
    bool calibrated;        // 是否标定
    double max_error;       // 标定误差
    int alg_type;           // 标定算法
    std::vector<CartesianPosition> robot_calibration_position;          // 机器人标定点
    std::vector<CartesianPosition> tracker_calibration_position;        // 喷枪标定点
    CartesianOrientation robot_calibration_orientation;                 // 机器人标定姿态
    CartesianOrientation tracker_calibration_orientation;               // 喷枪标定姿态
    Eigen::Matrix4d* position_calibration_matrix;
    Eigen::Matrix3d* orientation_offset_matrix;

    Json::Value to_json() const;
    void from_json(const Json::Value& val);

    Calibration_robotbase_2_trackerbase()
        :   calibrated(false), max_error(0.00), alg_type(0),
            position_calibration_matrix(new Eigen::Matrix4d()),
            orientation_offset_matrix(new Eigen::Matrix3d())
    {
        robot_calibration_position.clear();
        tracker_calibration_position.clear();
    }

    Calibration_robotbase_2_trackerbase(const Calibration_robotbase_2_trackerbase& other)
        : calibrated(other.calibrated),
          max_error(other.max_error),
          alg_type(other.alg_type),
          robot_calibration_position(other.robot_calibration_position),
          tracker_calibration_position(other.tracker_calibration_position),
          robot_calibration_orientation(other.robot_calibration_orientation),
          tracker_calibration_orientation(other.tracker_calibration_orientation),
          position_calibration_matrix(new Eigen::Matrix4d(*other.position_calibration_matrix)),
          orientation_offset_matrix(new Eigen::Matrix3d(*other.orientation_offset_matrix)) {}

    Calibration_robotbase_2_trackerbase& operator=(const Calibration_robotbase_2_trackerbase& other)
    {
        if (this == &other)
            return *this;

        delete position_calibration_matrix;
        delete orientation_offset_matrix;

        calibrated = other.calibrated;
        max_error = other.max_error;
        alg_type = other.alg_type;
        robot_calibration_position = other.robot_calibration_position;
        tracker_calibration_position = other.tracker_calibration_position;
        robot_calibration_orientation = other.robot_calibration_orientation;
        tracker_calibration_orientation = other.tracker_calibration_orientation;
        position_calibration_matrix = new Eigen::Matrix4d(*other.position_calibration_matrix);
        orientation_offset_matrix = new Eigen::Matrix3d(*other.orientation_offset_matrix);

        return *this;
    }

    ~Calibration_robotbase_2_trackerbase()
    {
        delete position_calibration_matrix;
        delete orientation_offset_matrix;
    }
};

struct Calibration_flange_2_tcp
{
    bool calibrated;
    int alg_type;
    std::vector<CartesianPose> flange_calibration_pose;     // 法兰盘标定点
    Eigen::Matrix4d* calibration_matrix;                    

    Json::Value to_json() const;
    void from_json(const Json::Value& val);

    Calibration_flange_2_tcp()
     :  calibrated(false), alg_type(0),
        calibration_matrix(new Eigen::Matrix4d())
    {
        flange_calibration_pose.clear();
    }

    Calibration_flange_2_tcp(const Calibration_flange_2_tcp& other)
        :   calibrated(other.calibrated),
            alg_type(other.alg_type),
            flange_calibration_pose(other.flange_calibration_pose),
            calibration_matrix(new Eigen::Matrix4d(*other.calibration_matrix)) {}

    Calibration_flange_2_tcp& operator=(const Calibration_flange_2_tcp& other)
    {
        if (this == &other)
            return *this;

        delete calibration_matrix;

        calibrated = other.calibrated;
        alg_type = other.alg_type;
        flange_calibration_pose = other.flange_calibration_pose;
        calibration_matrix = new Eigen::Matrix4d(*other.calibration_matrix);

        return *this;
    }

    ~Calibration_flange_2_tcp()
    {
        delete calibration_matrix;
    }
};

struct Calibration_tracker_2_tcp
{
    bool calibrated;
    int alg_type;
    std::vector<CartesianPosition> calibration_position;        // 追踪器标定点
    Eigen::Matrix4d* position_calibration_matrix;
    Eigen::Matrix3d* orientation_calibration_matrix;

    Json::Value to_json() const;
    void from_json(const Json::Value& val);

    Calibration_tracker_2_tcp()
     :  calibrated(false), alg_type(0),
        position_calibration_matrix(new Eigen::Matrix4d()),
        orientation_calibration_matrix(new Eigen::Matrix3d())
    {
        calibration_position.clear();
    }

    Calibration_tracker_2_tcp(const Calibration_tracker_2_tcp& other)
     :  calibrated(other.calibrated),
        alg_type(other.alg_type),
        calibration_position(other.calibration_position),
        position_calibration_matrix(new Eigen::Matrix4d(*other.position_calibration_matrix)),
        orientation_calibration_matrix(new Eigen::Matrix3d(*other.orientation_calibration_matrix)) {}

    Calibration_tracker_2_tcp& operator=(const Calibration_tracker_2_tcp& other)
    {
        if (this == &other)
            return *this;
        
        delete position_calibration_matrix;
        delete orientation_calibration_matrix;

        calibrated = other.calibrated;
        alg_type = other.alg_type;
        calibration_position = other.calibration_position;
        position_calibration_matrix = new Eigen::Matrix4d(*other.position_calibration_matrix);
        orientation_calibration_matrix = new Eigen::Matrix3d(*other.orientation_calibration_matrix);

        return *this;
    }

    ~Calibration_tracker_2_tcp()
    {
        delete position_calibration_matrix;
        delete orientation_calibration_matrix;
    }
};

struct CalibrationConfig
{
    Calibration_robotbase_2_trackerbase root2tracker;
    Calibration_flange_2_tcp flange2tcp;
    Calibration_tracker_2_tcp tracker2tcp;


    static CalibrationConfig from_file(const std::string& path);
    void to_file(const std::string& path) const;
};
