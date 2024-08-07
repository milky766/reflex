#ifndef RATE_LOOP_H
#define RATE_LOOP_H

#include <iostream>
#include <chrono>
#include <thread>

class RateLoop
{
public:
    RateLoop(int Hz, int duration);

    ~RateLoop();

    bool Sleep();

    uint64_t TimeStamp();

private:
    std::chrono::time_point<std::chrono::system_clock> start_;
    std::chrono::time_point<std::chrono::system_clock> curr_;
    int Hz_;
    uint64_t once_us_;
    int duration_;
};

#endif