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
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>  // POSIX mkdir
#include <sys/types.h>

#ifdef _WIN32
#include <direct.h>    // Windows mkdir
#endif
#include "json/json.h"

#define D_CONFIG_BASE_PATH                      "./config"
#define D_CONFIG_CALIBRATION_PATH               "./config/calibration.json"

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
    static bool ensure_directory_exists(const std::string& dir);
    Json::Value root_;
};