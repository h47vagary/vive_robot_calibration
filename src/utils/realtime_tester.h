#ifndef REALTIME_TESTER_H
#define REALTIME_TESTER_H

#include <windows.h>
#include <vector>
#include <functional>

class RealtimeTester
{
public:
    using TaskFunction = std::function<void(void)>;

    RealtimeTester(int period_ms, int test_duration_ms);

    // 设置任务函数（替换默认Sleep）
    void set_task_function(TaskFunction func);

    // 设置高优先级线程（默认true）
    void enable_high_priority(bool enable);

    // 开始测试
    void run_test();

private:
    int period_ms_;
    int test_duration_ms_;
    bool use_high_priority_;
    LARGE_INTEGER frequency_;
    std::vector<double> delays_;
    TaskFunction task_func_;

    void report_result();
    void set_thread_priority();
};

#endif // REALTIME_TESTER_H
