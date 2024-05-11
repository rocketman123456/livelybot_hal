#include "humanoid/ankle_fk.h"

#include <chrono>
#include <iostream>
#include <math.h>

int main()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    int count = 0;
    for (double mL = -30.0; mL < 30.0; mL += 1.0)
    {
        for (double mR = -30.0; mR < 30.0; mR += 1.0)
        {
            double d  = 69.8 / 2.0;
            double L1 = 25.0;
            double h1 = 112.0;
            double h2 = 65.0;

            auto  ankle_l   = ankle_fk(d, L1, h1, h2, mL / 180.0 * M_PI, mR / 180.0 * M_PI);
            float ankle_l_x = ankle_l[0];
            float ankle_l_y = ankle_l[1];
            count++;
        }
    }

    // double d  = 69.8 / 2.0;
    // double L1 = 25.0;
    // double h1 = 112.0;
    // double h2 = 65.0;

    // auto  ankle_l   = ankle_fk(d, L1, h1, h2, 0 / 180.0 * M_PI, 0 / 180.0 * M_PI);
    // float ankle_l_x = ankle_l[0];
    // float ankle_l_y = ankle_l[1];
    // count++;

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / static_cast<double>(count)
              << "[ms]" << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / static_cast<double>(count)
              << "[µs]" << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / static_cast<double>(count)
              << "[ns]" << std::endl;

    return 0;
}
