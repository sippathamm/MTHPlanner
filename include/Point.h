//
// Created by Sippawit Thammawiset on 14/1/2024 AD.
//

#ifndef POINT_H
#define POINT_H

#include <cmath>

namespace MTH
{
    /**
     * @brief A struct representing a 2D point with X and Y coordinates.
     */
    typedef struct APoint
    {
    public:
        double X; /**< X-coordinate of the point. */
        double Y; /**< Y-coordinate of the point. */

        /**
         * @brief Default constructor initializing X and Y to 0.0.
         */
        APoint () : X(0.0), Y(0.0) {}

        /**
         * @brief Constructor initializing X and Y with provided values.
         *
         * @param X X-coordinate.
         * @param Y Y-coordinate.
         */
        APoint (double X, double Y) : X(X), Y(Y) {}

        /**
         * @brief Copy constructor.
         *
         * @param AnotherPoint Another APoint instance to copy from.
         */
        APoint (const APoint &AnotherPoint)
        {
            X = AnotherPoint.X;
            Y = AnotherPoint.Y;
        }

        /**
         * @brief Overloaded addition operator to add two points.
         *
         * @param AnotherPoint Another point to add.
         *
         * @return Resultant point after addition.
         */
        APoint operator + (const APoint &AnotherPoint) const
        {
            return {X + AnotherPoint.X, Y + AnotherPoint.Y};
        }

        /**
         * @brief Overloaded subtraction operator to subtract two points.
         *
         * @param AnotherPoint Another point to subtract.
         *
         * @return Resultant point after subtraction.
         */
        APoint operator - (const APoint &AnotherPoint) const
        {
            return {X - AnotherPoint.X, Y - AnotherPoint.Y};
        }

        /**
         * @brief Overloaded multiplication operator to multiply two points.
         *
         * @param AnotherPoint Another point to multiply.
         *
         * @return Resultant point after multiplication.
         */
        APoint operator * (const APoint &AnotherPoint) const
        {
            return {X * AnotherPoint.X, Y * AnotherPoint.Y};
        }

        /**
         * @brief Overloaded multiplication operator to multiply a point by a constant.
         *
         * @param Constant Constant value to multiply.
         *
         * @return Resultant point after multiplication.
         */
        APoint operator * (double Constant) const
        {
            return {X * Constant, Y * Constant};
        }

        /**
         * @brief Overloaded division operator to divide a point by a constant.
         *
         * @param Constant Constant value to divide.
         *
         * @return Resultant point after division.
         */
        APoint operator / (double Constant) const
        {
            return {X / Constant, Y / Constant};
        }

        /**
         * @brief Overloaded equality operator to check if two points are equal.
         *
         * @param AnotherPoint Another point to compare.
         *
         * @return True if points are equal, false otherwise.
         */
        bool operator == (const APoint &AnotherPoint) const
        {
            return (X == AnotherPoint.X) && (Y == AnotherPoint.Y);
        }

        /**
         * @brief Overloaded inequality operator to check if two points are not equal.
         *
         * @param AnotherPoint Another point to compare.
         *
         * @return True if points are not equal, false otherwise.
         */
        bool operator != (const APoint &AnotherPoint) const
        {
            return !(*this == AnotherPoint);
        }
    } APoint;

    /**
     * @brief Compute the absolute value of each component of a point.
     *
     * @param Point Input point.
     *
     * @return Point with absolute values of X and Y coordinates.
     */
    APoint Absolute(const APoint &Point)
    {
        return {std::abs(Point.X), std::abs(Point.Y)};
    }
}

#endif //POINT_H