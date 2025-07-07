#pragma once
#include "calibration_config.h"
#include "task_executor.h"
#include <functional>


class CalibrationConfigAsync
{
public:
    // 异步读取配置
    static void load_async(const std::string& path,
                            CalibrationConfig* config_ptr = nullptr,
                            std::function<void(CalibrationConfig)> on_success = nullptr,
                            std::function<void()> on_fail = nullptr)
    {
        TaskExecutor::run([=]() 
        {
            CalibrationConfig temp_config;
            if (CalibrationConfig::from_file(path, temp_config))
            {
                if (config_ptr) 
                {
                    *config_ptr = std::move(temp_config);
                } 
                else if (on_success) 
                {
                    on_success(std::move(temp_config));
                }
            }
            else
            {
                if (on_fail) on_fail();
            }
        });
    }

    // 异步保存配置
    static void save_async(const std::string& path,
                           CalibrationConfig* config_ptr,
                           std::function<void()> on_success = nullptr,
                           std::function<void()> on_fail = nullptr)
    {
        TaskExecutor::run([=]() 
        {
            try {
                config_ptr->to_file(path);
                if (on_success) on_success();
            } catch (...) {
                if (on_fail) on_fail();
            }
        });
    }
};