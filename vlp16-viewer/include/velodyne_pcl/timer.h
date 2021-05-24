#pragma once
#include <chrono>
#include <iostream>

namespace velodyne_pcl
{
    class Timer
    {
        using clock_t = std::chrono::high_resolution_clock;
        using millsecond_t = std::chrono::milliseconds;

        std::chrono::time_point<clock_t> start_time = clock_t::now();
        std::string name_;
    public:
        Timer() = default;
        Timer(const std::string &name)
            : name_(std::move(name)) {}
        void elapsed()
        {
            std::chrono::time_point<clock_t> end_time = clock_t::now();
            auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            std::cout << name_ + " duration is "
                      << elapsedTime.count()
                      << " ms"
                      << std::endl;
        }
    };
};
