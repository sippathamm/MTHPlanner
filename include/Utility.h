//
// Created by Sippawit Thammawiset on 6/2/2024 AD.
//

#ifndef UTILITY_H
#define UTILITY_H

#include <random>

#define LETHAL_COST                             253

#define CLAMP(X, MIN, MAX)                      std::max(MIN, std::min(MAX, X))
#define IS_OUT_OF_BOUND(X, MIN, MAX)            X <= MIN || X >= MAX

namespace Optimizer
{
    typedef int PATH_TYPE;

    enum
    {
        FAILED = 0,
        SUCCESS = 1
    };

    enum
    {
        LINEAR = 0,
        CUBIC_SPLINE = 1
    };

    double GenerateRandom (double LowerBound = 0.0f, double UpperBound = 1.0f)
    {
        std::random_device Engine;
        std::uniform_real_distribution<double> RandomDistribution(0.0f, 1.0f);
        return LowerBound + RandomDistribution(Engine) * (UpperBound - LowerBound);
    }

    int GenerateRandomIndex (int Index)
    {
        std::random_device Engine;
        std::uniform_int_distribution<int> RandomDistribution(0, Index - 1);
        return RandomDistribution(Engine);
    }

    std::vector<double> LinearInterpolation (double Begin, double End, int Size)
    {
        std::vector<double> Result;

        if (Size == 0)
        {
            return Result;
        }

        if (Size == 1)
        {
            Result.push_back(Begin);
            return Result;
        }

        double Delta = (End - Begin) / (Size - 1);

        for (int i = 0; i < Size - 1; i++)
        {
            Result.push_back(Begin + Delta * i);
        }

        Result.push_back(End);

        return Result;
    }
}

#endif // UTILITY_H
