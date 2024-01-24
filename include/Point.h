//
// Created by Sippawit Thammawiset on 14/1/2024 AD.
//

#ifndef POINT_H
#define POINT_H

namespace Optimizer
{
    typedef struct APoint
    {
    public:
        APoint() : X(0.0f), Y(0.0f) {}
        APoint(double X, double Y) : X(X), Y(Y) {}

        double X;
        double Y;

        APoint operator + (const APoint &AnotherPoint) const
        {
            return {this->X + AnotherPoint.X, this->Y + AnotherPoint.Y};
        }

        APoint operator - (const APoint &AnotherPoint) const
        {
            return {this->X - AnotherPoint.X, this->Y - AnotherPoint.Y};
        }

        APoint operator * (const APoint &AnotherPoint) const
        {
            return {this->X * AnotherPoint.X, this->Y * AnotherPoint.Y};
        }

        APoint operator * (double Constant) const
        {
            return {this->X * Constant, this->Y * Constant};
        }

        bool operator == (const APoint &AnotherPoint) const
        {
            return (this->X == AnotherPoint.X) && (this->Y == AnotherPoint.Y);
        }

        bool operator != (const APoint &AnotherPoint) const
        {
            return !(*this == AnotherPoint);
        }
    } APoint;
}

#endif //POINT_H