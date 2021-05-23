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

    public:
        void elapsed()
        {
            std::chrono::time_point<clock_t> end_time = clock_t::now();
            auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            std::cout << elapsedTime.count()
                      << " ms"
                      << std::endl;
        }
    };
};
