//
// Created by Sippawit Thammawiset on 6/2/2024 AD.
//

#ifndef UTILITY_H
#define UTILITY_H

#include <random>

#define LETHAL_COST 253 /**< Cost value indicating a lethal area in the map. */

/**
 * @brief A macro to clamp a value between a minimum and a maximum.
 */
#define CLAMP(X, MIN, MAX)                      std::max(MIN, std::min(MAX, X))

/**
 * @brief A macro to check if a value is out of bounds.
 */
#define IS_OUT_OF_BOUND(X, MIN, MAX)            X < MIN || X > MAX

namespace MTH
{
    typedef int TRAJECTORY_TYPE;
    typedef int INITIAL_POSITION_TYPE;

    namespace STATE
    {
        /**
         * @brief An enum defining the states of the algorithm.
         */
        enum
        {
            FAILED = 0,
            SUCCESS = 1
        };
    }

    namespace TRAJECTORY
    {
        /**
         * @brief An enum defining trajectory types.
         */
        enum TRAJECTORY
        {
            LINEAR = 0, /**< Linear trajectory. */
            CUBIC_SPLINE = 1 /**< Cubic spline trajectory. */
        };
    }

    namespace INITIAL_POSITION
    {
        /**
         * @brief An enum defining initial position distribution types.
         */
        enum INITIAL_POSITION
        {
            DISTRIBUTED = 0, /**< Distributed initial positions. */
            LINEAR = 1, /**< Linear initial positions. */
            CIRCULAR = 2, /**< Circular initial positions. */
        };
    }

    /**
     * @brief Generate a random number within a specified range.
     *
     * @param LowerBound Lower bound of the range.
     * @param UpperBound Upper bound of the range.
     * @return Random number within the specified range.
     */
    double GenerateRandom (double LowerBound = 0.0, double UpperBound = 1.0)
    {
        std::random_device Engine;
        std::uniform_real_distribution<double> RandomDistribution(0.0, 1.0);
        return LowerBound + RandomDistribution(Engine) * (UpperBound - LowerBound);
    }

    /**
     * @brief Generate a random integer between 0 and N.
     *
     * @param N Upper bound of the range.
     * @return Random integer between 0 and N.
     */
    int GenerateRandom (int N)
    {
        std::random_device Engine;
        std::uniform_int_distribution<int> RandomDistribution(0, N);
        return RandomDistribution(Engine);
    }

    /**
     * @brief Perform linear interpolation between two values.
     *
     * @param Begin Starting value.
     * @param End Ending value.
     * @param Size Number of interpolated points (including Begin and End).
     *
     * @return Vector containing the interpolated values.
     */
    std::vector<double> LinearInterpolation (double Begin, double End, int Size)
    {
        std::vector<double> Result;

        // If Size is 0, return an empty vector.
        if (Size == 0)
        {
            return Result;
        }

        // If Size is 1, return a vector containing only Begin.
        if (Size == 1)
        {
            Result.push_back(Begin);
            return Result;
        }

        // Calculate the step size for interpolation.
        double Delta = (End - Begin) / (Size - 1);

        // Perform linear interpolation.
        for (int i = 0; i < Size - 1; ++i)
        {
            Result.push_back(Begin + Delta * i);
        }

        // Add the End value to the result vector.
        Result.push_back(End);

        return Result;
    }
}

#endif // UTILITY_H
