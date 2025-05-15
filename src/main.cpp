#include "vive_wrapper.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <windows.h>

int main()
{
    SetConsoleOutputCP(CP_UTF8);

    if (!vive_initialize())
    {
        std::cerr << "初始化 VIVE 失败！" << std::endl;
        return -1;
    }

    if (!vive_find_tracker())
    {
        std::cerr << "未找到 VIVE Tracker！" << std::endl;
        vive_shutdown();
        return -1;
    }

    std::cout << "请将 Tracker 放到原点位置后按回车..." << std::endl;
    std::cin.get();

    // 设置当前姿态为原点
    float x, y, z, qx, qy, qz, qw;
    uint64_t button;
    if (vive_get_pose(&x, &y, &z, &qx, &qy, &qz, &qw, &button))
    {
        vive_set_origin(x, y, z, qx, qy, qz, qw);
        std::cout << "原点设置完成。" << std::endl;
    }
    else
    {
        std::cerr << "获取初始姿态失败！" << std::endl;
        vive_shutdown();
        return -1;
    }

    // 开始记录到文件
    vive_start_logging("output.csv");

    // 循环获取相对位姿
    while(1)
    {
        float rx, ry, rz, rqx, rqy, rqz, rqw;
        if (vive_get_relative_pose(&rx, &ry, &rz, &rqx, &rqy, &rqz, &rqw, &button))
        {
            std::cout << "\r位置: [" << rx << ", " << ry << ", " << rz << "]"
                      << " 姿态: [" << rqx << ", " << rqy << ", " << rqz << "]"
                      << " Btn: " << button << "    " << std::flush;
        }
        else
        {
            std::cout << "Tracker 姿态无效。" << std::endl;
        }

        // 实时监听键盘上的 ESC 键
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
        {
            std::cout << "\n按下 ESC，提前结束录制。" << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 停止记录并释放资源
    vive_stop_logging();
    vive_shutdown();
    return 0;
}
