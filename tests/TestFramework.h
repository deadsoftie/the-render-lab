#pragma once
#include <cmath>
#include <iostream>

namespace Test
{
    inline int g_total = 0;
    inline int g_failures = 0;

    inline void Check(bool cond, const char* expr, const char* file, int line)
    {
        ++g_total;
        if (!cond)
        {
            ++g_failures;
            std::cerr << file << ":" << line << ": FAILED " << expr << "\n";
        }
    }

    inline void CheckNear(float a, float b, float eps, const char* expr, const char* file, int line)
    {
        ++g_total;
        if (std::abs(a - b) > eps)
        {
            ++g_failures;
            std::cerr << file << ":" << line << ": FAILED " << expr << " (" << a << " vs " << b
                       << ")\n";
        }
    }

    inline int Summarize()
    {
        std::cout << (g_total - g_failures) << "/" << g_total << " checks passed\n";
        return g_failures == 0 ? 0 : 1;
    }
}

#define CHECK(cond) ::Test::Check((cond), #cond, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, eps) ::Test::CheckNear((a), (b), (eps), #a " ~= " #b, __FILE__, __LINE__)
