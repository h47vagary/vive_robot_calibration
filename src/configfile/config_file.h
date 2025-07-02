/**
 * @file config_file.h
 * @brief 提供配置文件读写功能
 * @version 0.1
 * @date 2025-07-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once
#include <string>
#include "json/json.h"

#define D_CONFIG_CALIBRATION_PATH               "calibration.json"

#define READ_IF_MEMBER(json, key, var, type) if (json.isMember(key)) var = json[key].as##type()

class ConfigFile
{
public:
    ConfigFile();

    bool load(const std::string& file_path);
    bool save(const std::string& file_path) const;

    void set(const std::string& key, const std::string& value);
    std::string get(const std::string& key) const;
    std::string get_or(const std::string& key, const std::string& default_val) const;

    Json::Value& root();
    const Json::Value& root() const;

private:
    Json::Value root_;
};