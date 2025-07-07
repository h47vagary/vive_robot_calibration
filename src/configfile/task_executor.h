#pragma once

#include <functional>
#include <thread>

class TaskExecutor
{
public:
    static void run(std::function<void()> task)
    {
        std::thread(std::move(task)).detach();
    }
};