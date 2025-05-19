#include "msg_json_transfer.h"
#include <sstream>
#include <string>
#include "json/json.h"

int MsgStructTransfer::transfer_command(E_JSON_COMMAND user_command, int serial_id, std::string &json_str_out)
{
    Json::Value rootValue;
    rootValue["user_command"] = user_command;
    rootValue["serial_id"] = serial_id;

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    json_str_out = Json::writeString(builder, rootValue);
    return 0;
}

int MsgStructTransfer::transfer_mark_point(E_JSON_COMMAND user_command, int serial_id, int point_index, std::string &json_str_out)
{
    Json::Value root;
    root["user_command"] = user_command;
    root["serial_id"] = serial_id;

    root["point_index"] = point_index;

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    json_str_out = Json::writeString(builder, root);
    return 0;
}

int MsgStructTransfer::transfer_rob_pose(E_JSON_COMMAND user_command, int serial_id, int point_index, CartesianPose pose, std::string &json_str_out)
{
    Json::Value root;
    root["user_command"] = user_command;
    root["serial_id"] = serial_id;

    root["point_index"] = point_index;
    root["position"]["x"] = pose.position.x;
    root["position"]["y"] = pose.position.y;
    root["position"]["z"] = pose.position.z;
    root["orientation"]["A"] = pose.orientation.A;
    root["orientation"]["B"] = pose.orientation.B;
    root["orientation"]["C"] = pose.orientation.C;

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    json_str_out = Json::writeString(builder, root);
    return 0;
}

int MsgStructTransfer::transfer_diy_message(E_JSON_COMMAND user_command, int serial_id, const std::string &message, std::string &json_str_out)
{
    Json::Value root;
    root["user_command"] = user_command;
    root["serial_id"] = serial_id;

    root["message"] = message;
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    json_str_out = Json::writeString(builder, root);
    return 0;
}

int MsgStructTransfer::transfer_robot_compute_result(E_JSON_COMMAND user_command, int serial_id, double result, std::string &json_str_out)
{
    Json::Value root;
    root["user_command"] = user_command;
    root["serial_id"] = serial_id;
    root["result"] = result;
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    json_str_out = Json::writeString(builder, root);
    return 0;
}

int MsgJsonTransfer::transfer_command(const std::string &json_str_in, E_JSON_COMMAND &command_out, int &serial_id_out)
{
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;

    std::istringstream iss(json_str_in);
    if (Json::parseFromStream(builder, iss, &root, &errs)) {
        if (!root.isObject())
            return -1;

        command_out = static_cast<E_JSON_COMMAND>(root["user_command"].asInt());
        serial_id_out = root["serial_id"].asInt();
        return 0;
    }
    return -1;
}

int MsgJsonTransfer::transfer_mark_point(const std::string &json_str_in, int &point_index)
{
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;

    std::istringstream iss(json_str_in);
    if (Json::parseFromStream(builder, iss, &root, &errs)) {
        if (!root.isObject())
            return -1;
        point_index = root["point_index"].asInt();
        return 0;
    }
    return -1;
}

int MsgJsonTransfer::transfer_rob_pose(const std::string &json_str_in, int &point_index, CartesianPose &pose)
{
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;

    std::istringstream iss(json_str_in);
    if (Json::parseFromStream(builder, iss, &root, &errs)) {
        if (!root.isObject())
            return -1;
        point_index = root["point_index"].asInt();
        pose.position.x = root["position"]["x"].asDouble();
        pose.position.y = root["position"]["y"].asDouble();
        pose.position.z = root["position"]["z"].asDouble();
        pose.orientation.A = root["orientation"]["A"].asDouble();
        pose.orientation.B = root["orientation"]["B"].asDouble();
        pose.orientation.C = root["orientation"]["C"].asDouble();
        return 0;
    }
    return -1;
}

int MsgJsonTransfer::transfer_diy_message(const std::string &json_str_in, std::string &message_out)
{
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;

    std::istringstream iss(json_str_in);
    if (Json::parseFromStream(builder, iss, &root, &errs)) {
        if (!root.isObject())
            return -1;
        message_out = root["message"].asString();
        return 0;
    }
    return -1;
}

int MsgJsonTransfer::transfer_robot_compute_result(const std::string &json_str_in, double &result_out)
{
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;

    std::istringstream iss(json_str_in);
    if (Json::parseFromStream(builder, iss, &root, &errs)) {
        if (!root.isObject())
            return -1;
        result_out = root["result"].asDouble();
        return 0;
    }
    return -1;
}
