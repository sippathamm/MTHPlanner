//
// Created by Sippawit Thammawiset on 7/2/2024 AD.
//

#ifndef DEBUG_H
#define DEBUG_H

#include "Point.h"

#include <vector>
#include <ostream>

/**
 * @brief Overloaded output stream operator to print a vector of points.
 *
 * @param OS Output stream.
 * @param Position Vector of points to be printed.
 *
 * @return Reference to the output stream.
 */
std::ostream & operator << (std::ostream &OS, const std::vector<MTH::APoint> &Position)
{
    OS << "[";
    for (const auto &i : Position)
    {
        OS << "(" << i.X << ", " << i.Y << "), ";
    }
    OS << "]";

    return OS;
}

/**
 * @brief Overloaded output stream operator to print a vector of any type.
 *
 * @tparam T Type of elements in the vector.
 * @param OS Output stream.
 * @param Vector Vector to be printed.
 *
 * @return Reference to the output stream.
 */
template <typename T>
std::ostream & operator << (std::ostream &OS, const std::vector<T> &Vector)
{
    OS << "[";
    for (const auto &i : Vector)
    {
        OS << i << ", ";
    }
    OS << "]";

    return OS;
}

/**
 * @brief Overloaded output stream operator to print a point.
 *
 * @param OS Output stream.
 * @param Point Point to be printed.
 *
 * @return Reference to the output stream.
 */
std::ostream & operator << (std::ostream &OS, const MTH::APoint &Point)
{
    OS << "(" << Point.X << ", " << Point.Y << ")";

    return OS;
}

#endif // DEBUG_H
