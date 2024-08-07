#include <rate_loop.h>

RateLoop::RateLoop(int Hz, int duration) {
    Hz_ = Hz;
    duration_ = duration;
    once_us_ = 1e6 / Hz;
    start_ = std::chrono::system_clock::now();
    curr_ = start_;
}

RateLoop::~RateLoop()
{
    // if (auto_sleep) { Sleep(); }
}



bool RateLoop::Sleep()
{
    // 获取经过的时间
    auto now = std::chrono::system_clock::now();
    auto elapse = std::chrono::duration_cast<std::chrono::microseconds>(now - curr_).count();
    auto lasting = std::chrono::duration_cast<std::chrono::microseconds>(now - start_).count();
    // 计算本轮循环剩余的时间
    uint64_t remain_us = once_us_ - elapse;
    if (remain_us >= 0)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(remain_us));
    }
    else {
        std::cerr << "执行超时\n";
    }
    // 重置计算器
    curr_ = std::chrono::system_clock::now();
    return lasting <= duration_ * 1e6;
}

uint64_t RateLoop::TimeStamp() {
    return std::chrono::duration_cast<std::chrono::seconds>(start_.time_since_epoch()).count();
    //return timeStamp
}