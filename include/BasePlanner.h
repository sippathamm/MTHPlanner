//
// Created by Sippawit Thammawiset on 17/2/2024 AD.
//

#ifndef BASE_PLANNER_H
#define BASE_PLANNER_H

#include "Print.h"
#include "Utility.h"
#include "Point.h"
#include "spline.h"

#include <iostream>
#include <vector>
#include <random>
#include <cmath>

namespace MTH
{
    class ABasePlanner
    {
    public:
        /**
         * @brief Constructor.
         *
         * @param LowerBound Lower bound of the search space.
         * @param UpperBound Upper bound of the search space.
         * @param MaximumIteration Maximum number of iterations.
         * @param NPopulation Population size.
         * @param NBreakpoint Number of breakpoints.
         * @param NWaypoint Number of waypoints.
         * @param TrajectoryType Type of trajectory.
         */
        ABasePlanner (const APoint &LowerBound, const APoint &UpperBound,
                      int MaximumIteration, int NPopulation, int NBreakpoint, int NWaypoint,
                      TRAJECTORY_TYPE TrajectoryType = TRAJECTORY::CUBIC_SPLINE) :
                      LowerBound_(LowerBound),
                      UpperBound_(UpperBound),
                      MaximumIteration_(MaximumIteration),
                      NPopulation_(NPopulation),
                      NBreakpoint_(NBreakpoint),
                      NWaypoint_(NWaypoint),
                      TrajectoryType_(TrajectoryType)
        {

        }

        /**
         * @brief Destructor.
         */
        virtual ~ABasePlanner() = default;

        /**
         * @brief Create a plan.
         *
         * @param CostMap Pointer to the cost map.
         * @param Start Start point of the path.
         * @param Goal Goal point of the path.
         * @param Waypoint Vector to store the generated waypoints.
         *
         * @return SUCCESS if the plan is successfully created, FAILED otherwise.
         */
        virtual bool CreatePlan (const unsigned char *CostMap,
                                 const APoint &Start,
                                 const APoint &Goal,
                                 std::vector<APoint> &Path) = 0;

        /**
         * @brief Get the global best position found by the algorithm.
         *
         * @return Global best position.
         */
        std::vector<APoint> GetGlobalBestPosition () const
        {
            return this->GlobalBestPosition_;
        }

        /**
         * @brief Get the global best cost found by the algorithm.
         *
         * @return Global best cost.
         */
        double GetGlobalBestCost () const
        {
            return this->GlobalBestCost_;
        }

        /**
         * @brief Get the length of the generated path.
         *
         * @return Length of the generated path.
         */
        double GetPathLength () const
        {
            return this->PathLength_;
        }

        /**
         * @brief Get the convergence history.
         *
         * @return Vector containing the convergence history.
         */
        std::vector<double> GetConvergence ()
        {
            return this->Convergence_;
        }

        /**
         * @brief Clear the planner's state.
         */
        virtual void Clear () = 0;


    protected:
        APoint LowerBound_; /**< Lower bound of the search space. */
        APoint UpperBound_; /**< Upper bound of the search space. */
        int MaximumIteration_; /**< Maximum number of iterations. */
        int NPopulation_; /**< Population size. */
        int NBreakpoint_; /**< Number of breakpoints. */
        int NWaypoint_; /**< Number of waypoints. */

        const unsigned char *CostMap_ = nullptr; /**< Pointer to the cost map. */
        const APoint *Start_ = nullptr; /**< Pointer to the start point. */
        const APoint *Goal_ = nullptr; /**< Pointer to the goal point. */

        float ObstacleCostFactor_ = 1.0; /**< Factor for obstacle cost. */
        float CostScalingFactor_ = 1.0; /**< Scaling factor for cost. */
        float PenaltyScalingFactor_ = 1000.0; /**< Scaling factor for penalty. */

        APoint Range_; /**< Range of the search space. */
        int N_{}; /**< Size of the search space. */

        std::vector<APoint> GlobalBestPosition_; /**< Global best position found by the algorithm. */
        double GlobalBestCost_ = 2e10; /**< Global best cost found by the algorithm. */

        double PathLength_ = 0.0; /**< Length of the generated path. */

        TRAJECTORY_TYPE TrajectoryType_; /**< Type of trajectory. */
        std::vector<double> Convergence_; /**< Convergence history. */

        /**
         * @brief Generate a distributed position within the search space.
         *
         * @return Distributed position within the search space.
         */
        APoint GenerateDistributedPosition () const
        {
            APoint RandomPosition;

            // Generate random values within the bounds for both dimensions.
            RandomPosition.X = GenerateRandom(this->LowerBound_.X, this->UpperBound_.X);
            RandomPosition.Y = GenerateRandom(this->LowerBound_.Y, this->UpperBound_.Y);

            return RandomPosition;
        }

        /**
         * @brief Generate a circular position within the search space.
         *
         * @return Circular position within the search space.
         */
        APoint GenerateCircularPosition ()
        {
            static double Radius = std::hypot(this->Start_->X - this->Goal_->X,
                                              this->Start_->Y - this->Goal_->Y) * 0.5;
            static APoint Center = (*this->Goal_ + *this->Start_) * 0.5;

            // Generate a random angle and radius within the circular area.
            double Angle = GenerateRandom(0.0, 2.0 * M_PI);
            double RandomRadius = std::sqrt(GenerateRandom(0.0, 1.0)) * Radius;

            APoint RandomPosition;

            // Calculate the position coordinates based on the angle and radius.
            RandomPosition.X = Center.X + RandomRadius * std::cos(Angle);
            RandomPosition.Y = Center.Y + RandomRadius * std::sin(Angle);

            // Ensure the generated position is within the search space bounds.
            RandomPosition.X = CLAMP(RandomPosition.X, this->LowerBound_.X, this->UpperBound_.X);
            RandomPosition.Y = CLAMP(RandomPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y);

            return RandomPosition;
        }

        /**
         * @brief Generate a linear position within the search space based on the given breakpoint index.
         *
         * @param BreakpointIndex Index of the breakpoint.
         *
         * @return Linear position within the search space.
         */
        APoint GenerateLinearPosition (int BreakpointIndex)
        {
            // Calculate the ratio of the current breakpoint index to the total number of breakpoints.
            double M = static_cast<double>(BreakpointIndex) / (this->NBreakpoint_ - 1);

            // Interpolate between the start and goal points based on the calculated ratio.
            APoint RandomPosition = *this->Start_ + (*this->Goal_ - *this->Start_) * M;

            // Add random noise to the position within a small range.
            RandomPosition.X += GenerateRandom(-this->Range_.X * 0.05, this->Range_.X * 0.05);
            RandomPosition.Y += GenerateRandom(-this->Range_.Y * 0.05, this->Range_.Y * 0.05);

            return RandomPosition;
        }

        /**
         * @brief Compute the objective function value for a given particle position.
         *
         * @param Position The position of particle.
         *
         * @return The objective function value.
         */
        double ObjectiveFunction (const std::vector<APoint> &Position)
        {
            double Length = 0.0;
            std::vector<APoint> Waypoint;

            // Generate the path based on the trajectory type.
            switch (this->TrajectoryType_)
            {
                case TRAJECTORY::LINEAR:
                    LinearPath(Length, Waypoint, Position);
                    break;

                case TRAJECTORY::CUBIC_SPLINE:
                    CubicSplinePath(Length, Waypoint, Position);
                    break;

                default:
                    CubicSplinePath(Length, Waypoint, Position);
            }

            // Calculate the penalty and apply scaling factors to compute the cost.
            double Error = Penalty(Waypoint) * this->PenaltyScalingFactor_;
            double Cost = this->CostScalingFactor_ * Length * (1.0 + Error);

            return Cost;
        }

        /**
         * @brief Construct the breakpoint coordinates from the given particle position.
         *
         * @param Position The position of particle.
         *
         * @return A pair of vectors containing X and Y coordinates of the breakpoints.
         */
        std::pair<std::vector<double>, std::vector<double>> ConstructBreakpoint (const std::vector<APoint> &Position) const
        {
            std::vector<double> X(this->NBreakpoint_ + 2);
            std::vector<double> Y(this->NBreakpoint_ + 2);

            // Set the start point coordinates as the first breakpoint.
            X.front() = this->Start_->X;
            Y.front() = this->Start_->Y;

            // Set the particle position coordinates as breakpoints.
            for (int BreakpointIndex = 1; BreakpointIndex <= this->NBreakpoint_; ++BreakpointIndex)
            {
                X[BreakpointIndex] = Position[BreakpointIndex - 1].X;
                Y[BreakpointIndex] = Position[BreakpointIndex - 1].Y;
            }

            // Set the goal point coordinates as the last breakpoint.
            X.back() = this->Goal_->X;
            Y.back() = this->Goal_->Y;

            return {X, Y};
        }

        /**
         * @brief Compute the linear path and its length.
         *
         * @param Length Reference to store the length of the linear path.
         * @param Waypoint Reference to store the waypoints of the linear path.
         * @param Position The position of particle.
         */
        void LinearPath (double &Length,
                         std::vector<APoint> &Waypoint,
                         const std::vector<APoint> &Position) const
        {
            // Construct the breakpoint from the particle's position.
            auto Breakpoint = ConstructBreakpoint(Position);
            auto X = Breakpoint.first;
            auto Y = Breakpoint.second;

            // Determine the number of interpolation points between each pair of breakpoints.
            int NInterpolationPoint = (this->NWaypoint_ - 1) / (this->NBreakpoint_ + 1);

            // Clear the waypoint vector and add the start point.
            Waypoint.clear();
            Waypoint.emplace_back(X.front(), Y.front());

            // Generate linearly interpolated points between each pair of breakpoints.
            for (int i = 1; i < this->NBreakpoint_ + 2; ++i)
            {
                std::vector<double> InterpolationX = LinearInterpolation(X[i - 1], X[i], NInterpolationPoint);
                std::vector<double> InterpolationY = LinearInterpolation(Y[i - 1], Y[i], NInterpolationPoint);

                for (int j = 0; j < NInterpolationPoint; ++j)
                {
                    Waypoint.emplace_back(InterpolationX[j], InterpolationY[j]);
                }
            }

            // Calculate the length of the linear path.
            CalculateLength(Length, Waypoint);
        }

        /**
         * @brief Compute the cubic spline path and its length.
         *
         * @param Length Reference to store the length of the cubic spline path.
         * @param Waypoint Reference to store the waypoints of the cubic spline path.
         * @param Position The position of particle.
         */
        void CubicSplinePath (double &Length,
                              std::vector<APoint> &Waypoint,
                              const std::vector<APoint> &Position) const
        {
            // Construct the breakpoint from the particle's position.
            auto Breakpoint = ConstructBreakpoint(Position);
            auto X = Breakpoint.first;
            auto Y = Breakpoint.second;

            // Generate linearly interpolated points for spline calculation.
            std::vector<double> Interpolation = LinearInterpolation(0.0, 1.0, this->NBreakpoint_ + 2);

            // Compute cubic spline interpolation for X and Y coordinates.
            tk::spline CubicSplineX(Interpolation, X);
            tk::spline CubicSplineY(Interpolation, Y);

            // Generate linearly interpolated points for waypoint calculation.
            Interpolation = LinearInterpolation(0.0, 1.0, this->NWaypoint_);

            // Clear the waypoint vector.
            Waypoint.clear();

            // Compute waypoints using cubic spline interpolation.
            for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; ++WaypointIndex)
            {
                double XInterpolation = CubicSplineX(Interpolation[WaypointIndex]);
                double YInterpolation = CubicSplineY(Interpolation[WaypointIndex]);

                Waypoint.emplace_back(XInterpolation, YInterpolation);
            }

            // Calculate the length of the cubic spline path.
            CalculateLength(Length, Waypoint);
        }

        /**
         * @brief Calculate the length of a path defined by waypoints.
         *
         * @param Length Reference to store the length of the path.
         * @param Waypoint Vector containing the waypoints of the path.
         */
        void CalculateLength (double &Length, const std::vector<APoint> &Waypoint) const
        {
            for (int WaypointIndex = 1; WaypointIndex < this->NWaypoint_; ++WaypointIndex)
            {
                double DX = Waypoint[WaypointIndex].X - Waypoint[WaypointIndex - 1].X;
                double DY = Waypoint[WaypointIndex].Y - Waypoint[WaypointIndex - 1].Y;
                Length += std::hypot(DX, DY);
            }
        }

        /**
         * @brief Convert XY coordinates to an index in a 1D array.
         *
         * @param X The X coordinate.
         * @param Y The Y coordinate.
         *
         * @return The index corresponding to the given XY coordinates.
         */
        int XYToIndex (int X, int Y) const
        {
            return Y * static_cast<int>(this->Range_.X) + X;
        }

        /**
         * @brief Calculate the penalty based on the provided waypoints.
         *
         * @param Waypoint Vector containing the waypoints.
         *
         * @return The penalty value.
         */
        double Penalty (const std::vector<APoint> &Waypoint) const
        {
            int BreakpointIndex = 0;
            int NInterpolationPoint = (this->NWaypoint_ - 1) / (this->NBreakpoint_ + 1);
            double Penalty = 0.0;

            // Iterate through waypoints
            for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; ++WaypointIndex)
            {
                // Convert waypoint coordinates to integer indices
                int X = static_cast<int>(Waypoint[WaypointIndex].X);
                int Y = static_cast<int>(Waypoint[WaypointIndex].Y);

                // Convert 2D indices to 1D index
                int Index = XYToIndex(X, Y);

                // Check if index is out of bounds or if there's an obstacle at the waypoint
                if (Index < 0 || Index >= this->N_ ||
                    this->CostMap_[Index] >= static_cast<int>(this->ObstacleCostFactor_ * LETHAL_COST))
                {
                    // Penalize based on whether the waypoint is a breakpoint or not
                    if (WaypointIndex == BreakpointIndex)
                    {
                        Penalty += 100.0;
                    }
                    else
                    {
                        Penalty += 20.0;
                    }
                }

                // Update breakpoint index
                if (WaypointIndex % NInterpolationPoint == 0)
                {
                    BreakpointIndex += NInterpolationPoint;
                }
            }

            return Penalty;
        }
    };
} // MTH

#endif // BASE_PLANNER_H
