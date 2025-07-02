#include "json_utils.h"

Json::Value eigen_matrix_to_json(const Eigen::MatrixXd& mat)
{
    Json::Value json_array(Json::arrayValue);
    for (int i = 0; i < mat.rows(); ++i) 
    {
        Json::Value row(Json::arrayValue);
        for (int j = 0; j < mat.cols(); ++j) 
        {
            row.append(mat(i, j));
        }
        json_array.append(row);
    }
    return json_array;
}

Eigen::MatrixXd json_to_eigen_matrix(const Json::Value& val)
{
    if (!val.isArray() || val.size() == 0 || !val[0].isArray())
        return Eigen::MatrixXd();

    int rows = val.size();
    int cols = val[0].size();
    Eigen::MatrixXd mat(rows, cols);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            mat(i, j) = val[i][j].asDouble();
    return mat;
}

Json::Value cartesian_position_to_json(const CartesianPosition& pos)
{
    Json::Value val;
    val["x"] = pos.x;
    val["y"] = pos.y;
    val["z"] = pos.z;
    return val;
}

CartesianPosition json_to_cartesian_position(const Json::Value& val)
{
    CartesianPosition p;
    p.x = val["x"].asDouble();
    p.y = val["y"].asDouble();
    p.z = val["z"].asDouble();
    return p;
}

Json::Value cartesian_orientation_to_json(const CartesianOrientation& ori)
{
    Json::Value val;
    val["A"] = ori.A;
    val["B"] = ori.B;
    val["C"] = ori.C;
    return val;
}

CartesianOrientation json_to_cartesian_orientation(const Json::Value& val)
{
    CartesianOrientation ori;
    ori.A = val["A"].asDouble();
    ori.B = val["B"].asDouble();
    ori.C = val["C"].asDouble();
    return ori;
}
