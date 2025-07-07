#pragma once
#include "json/json.h"
#include "Eigen/Dense"
#include "utils.h"

// Eigen 矩阵转 JSON 数组
Json::Value eigen_matrix_to_json(const Eigen::MatrixXd& mat);
Eigen::MatrixXd json_to_eigen_matrix(const Json::Value& val);

// CartesianPosition
Json::Value cartesian_position_to_json(const CartesianPosition& pos);
CartesianPosition json_to_cartesian_position(const Json::Value& val);

// CartesianOrientation
Json::Value cartesian_orientation_to_json(const CartesianOrientation& ori);
CartesianOrientation json_to_cartesian_orientation(const Json::Value& val);

Json::Value eigen_vector_to_json(const Eigen::VectorXd& vec);
Eigen::VectorXd json_to_eigen_vector(const Json::Value& val);