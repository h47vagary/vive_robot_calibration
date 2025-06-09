#include "realtime_tester.h"
#include <iostream>
#include <iomanip>

RealtimeTester::RealtimeTester(int period_ms, int test_duration_ms)
    : period_ms_(period_ms),
      test_duration_ms_(test_duration_ms),
      use_high_priority_(true),
      task_func_([]() { Sleep(4); }) // 默认任务：Sleep 4ms
{
    QueryPerformanceFrequency(&frequency_);
}

void RealtimeTester::set_task_function(TaskFunction func)
{
    task_func_ = func;
}

void RealtimeTester::enable_high_priority(bool enable)
{
    use_high_priority_ = enable;
}

void RealtimeTester::set_thread_priority()
{
    if (use_high_priority_)
    {
        HANDLE hThread = GetCurrentThread();
        SetThreadPriority(hThread, THREAD_PRIORITY_HIGHEST);
    }
}

void RealtimeTester::run_test()
{
    set_thread_priority();

    LARGE_INTEGER start_time, now;
    int iterations = test_duration_ms_ / period_ms_;
    delays_.clear();
    delays_.reserve(iterations);

    QueryPerformanceCounter(&start_time);
    LONGLONG expected_time = start_time.QuadPart;

    for (int i = 0; i < iterations; ++i)
    {
        if (task_func_) task_func_();  // 调用自定义函数

        QueryPerformanceCounter(&now);
        LONGLONG delay_ticks = now.QuadPart - expected_time;
        double delay_ms = (delay_ticks * 1000.0) / frequency_.QuadPart;
        delays_.push_back(delay_ms);

        expected_time += (period_ms_ * frequency_.QuadPart) / 1000;
    }

    report_result();
}

void RealtimeTester::report_result()
{
    if (delays_.empty()) return;

    double min_delay = delays_[0];
    double max_delay = delays_[0];
    double sum = 0.0;

    for (double d : delays_)
    {
        if (d < min_delay) min_delay = d;
        if (d > max_delay) max_delay = d;
        sum += d;
    }

    double avg_delay = sum / delays_.size();
    double jitter = max_delay - min_delay;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Realtime Test Result:\n";
    std::cout << "  Period(ms):       " << period_ms_ << "\n";
    std::cout << "  Samples:          " << delays_.size() << "\n";
    std::cout << "  Avg Delay(ms):    " << avg_delay << "\n";
    std::cout << "  Min Delay(ms):    " << min_delay << "\n";
    std::cout << "  Max Delay(ms):    " << max_delay << "\n";
    std::cout << "  Jitter(ms):       " << jitter << "\n";
}
