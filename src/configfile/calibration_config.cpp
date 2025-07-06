#include "calibration_config.h"
#include "config_file.h"
#include "json_utils.h"

bool CalibrationConfig::from_file(const std::string& path, CalibrationConfig& calbration_config)
{
    ConfigFile file;
    if (!file.load(path))
    {
        std::cerr << "Failed to load calibration config file: " << path << std::endl;
        return false;
    }

    calbration_config.root2tracker.from_json(file.root()["root2tracker"]);
    calbration_config.flange2tcp.from_json(file.root()["flange2tcp"]);
    calbration_config.tracker2tcp.from_json(file.root()["tracker2tcp"]);

    return true;
}

void CalibrationConfig::to_file(const std::string& path) const
{
    ConfigFile file;
    file.root()["root2tracker"] = root2tracker.to_json();
    file.root()["flange2tcp"] = flange2tcp.to_json();
    file.root()["tracker2tcp"] = tracker2tcp.to_json();

    if (!file.save(path))
        std::cerr << "Failed to save calibration config file: " << path << std::endl;
}

Json::Value Calibration_robotbase_2_trackerbase::to_json() const
{
    Json::Value val;
    val["calibrated"] = calibrated;
    val["max_error"] = max_error;
    val["alg_type"] = alg_type;

    for (const auto& p : robot_calibration_position)
        val["robot_calibration_position"].append(cartesian_position_to_json(p));

    for (const auto& p : tracker_calibration_position)
        val["tracker_calibration_position"].append(cartesian_position_to_json(p));

    val["robot_calibration_orientation"] = cartesian_orientation_to_json(robot_calibration_orientation);
    val["tracker_calibration_orientation"] = cartesian_orientation_to_json(tracker_calibration_orientation);

    val["position_calibration_matrix"] = eigen_matrix_to_json(*position_calibration_matrix);
    val["orientation_offset_matrix"] = eigen_matrix_to_json(*orientation_offset_matrix);
    return val;
}

void Calibration_robotbase_2_trackerbase::from_json(const Json::Value &val)
{
    READ_IF_MEMBER(val, "calibrated", calibrated, Bool);
    READ_IF_MEMBER(val, "max_error", max_error, Double);
    READ_IF_MEMBER(val, "alg_type", alg_type, Int);

    robot_calibration_position.clear();
    for (auto& p : val["robot_calibration_position"])
        robot_calibration_position.push_back(json_to_cartesian_position(p));

    tracker_calibration_position.clear();
    for (auto& p : val["tracker_calibration_position"])
        tracker_calibration_position.push_back(json_to_cartesian_position(p));

    robot_calibration_orientation = json_to_cartesian_orientation(val["robot_calibration_orientation"]);
    tracker_calibration_orientation = json_to_cartesian_orientation(val["tracker_calibration_orientation"]);

    *position_calibration_matrix = json_to_eigen_matrix(val["position_calibration_matrix"]);
    *orientation_offset_matrix = json_to_eigen_matrix(val["orientation_offset_matrix"]);
}

Json::Value Calibration_flange_2_tcp::to_json() const
{
    Json::Value val;
    val["calibrated"] = calibrated;
    val["alg_type"] = alg_type;

    for (const auto& p : flange_calibration_pose)
    {
        Json::Value pose;
        pose["position"]["x"] = p.position.x;
        pose["position"]["y"] = p.position.y;
        pose["position"]["z"] = p.position.z;
        pose["orientation"]["A"] = p.orientation.A;
        pose["orientation"]["B"] = p.orientation.B;
        pose["orientation"]["C"] = p.orientation.C;
        val["flange_calibration_pose"].append(pose);
    }

    val["calibration_matrix"] = eigen_matrix_to_json(*calibration_matrix);
    return val;
}

void Calibration_flange_2_tcp::from_json(const Json::Value &val)
{
    READ_IF_MEMBER(val, "calibrated", calibrated, Bool);
    READ_IF_MEMBER(val, "alg_type", alg_type, Int);

    flange_calibration_pose.clear();
    for (const auto& p : val["flange_calibration_pose"])
    {
        CartesianPose pose;
        pose.position.x = p["position"]["x"].asDouble();
        pose.position.y = p["position"]["y"].asDouble();
        pose.position.z = p["position"]["z"].asDouble();
        pose.orientation.A = p["orientation"]["A"].asDouble();
        pose.orientation.B = p["orientation"]["B"].asDouble();
        pose.orientation.C = p["orientation"]["C"].asDouble();
        flange_calibration_pose.push_back(pose);
    }

    *calibration_matrix = json_to_eigen_matrix(val["calibration_matrix"]);
}

Json::Value Calibration_tracker_2_tcp::to_json() const
{
    Json::Value val;
    val["calibrated"] = calibrated;
    val["alg_type"] = alg_type;

    for (const auto& p : calibration_position)
        val["calibration_position"].append(cartesian_position_to_json(p));

    val["position_calibration_matrix"] = eigen_matrix_to_json(*position_calibration_matrix);
    val["orientation_calibration_matrix"] = eigen_matrix_to_json(*orientation_calibration_matrix);
    return val;
}

void Calibration_tracker_2_tcp::from_json(const Json::Value &val)
{
    READ_IF_MEMBER(val, "calibrated", calibrated, Bool);
    READ_IF_MEMBER(val, "alg_type", alg_type, Int);

    calibration_position.clear();
    for (const auto& p : val["calibration_position"])
        calibration_position.push_back(json_to_cartesian_position(p));

    *position_calibration_matrix = json_to_eigen_matrix(val["position_calibration_matrix"]);
    *orientation_calibration_matrix = json_to_eigen_matrix(val["orientation_calibration_matrix"]);
}
