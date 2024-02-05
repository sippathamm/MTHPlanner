//
// Created by Sippawit Thammawiset on 3/2/2024 AD.
//

#ifndef PLANNER_H
#define PLANNER_H 

#include "Point.h"
#include "spline.h"

#include <iostream>
#include <vector>
#include <random>
#include <cmath>

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

    typedef struct AParticle
    {
        AParticle () : BestFitnessValue((double)INFINITY), FitnessValue(0.0f) {}

        std::vector<APoint> Position;
        std::vector<APoint> Velocity;
        double FitnessValue;

        std::vector<APoint> BestPosition;
        double BestFitnessValue;

        std::vector<APoint> Feedback;
    } AParticle;

    class AGlobalPlanner
    {
    public:
        AGlobalPlanner (APoint LowerBound, APoint UpperBound,
                        int MaximumIteration, int NPopulation, int NBreakpoint, int NWaypoint,
                        double SocialCoefficient = 1.5f, double CognitiveCoefficient = 1.5f,
                        double VelocityFactor = 0.5f,
                        PATH_TYPE PathType = CUBIC_SPLINE,
                        bool Log = true) :
                        LowerBound_(LowerBound),
                        UpperBound_(UpperBound),
                        MaximumIteration_(MaximumIteration),
                        NPopulation_(NPopulation),
                        NBreakpoint_(NBreakpoint),
                        NWaypoint_(NWaypoint),
                        SocialCoefficient_(SocialCoefficient),
                        CognitiveCoefficient_(CognitiveCoefficient),
                        VelocityFactor_(VelocityFactor),
                        PathType_(PathType),
                        Log_(Log)
        {

        }

        ~AGlobalPlanner () = default;

        bool CreatePlan (const unsigned char *CostMap,
                         const APoint &Start,
                         const APoint &Goal,
                         std::vector<APoint> &Path)
        {
            // Initialize
            this->Population_ = std::vector<AParticle> (this->NPopulation_);

            this->Range_ = this->UpperBound_ - this->LowerBound_;
            this->N_ = static_cast<int>(this->Range_.X * this->Range_.Y);

            this->MaximumVelocity_ = (this->UpperBound_ - this->LowerBound_) * this->VelocityFactor_;
            this->MinimumVelocity_ = MaximumVelocity_ * -1.0f;

            for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
            {
                auto *CurrentPopulation = &this->Population_[PopulationIndex];

                std::vector<APoint> Position(this->NBreakpoint_);
                std::vector<APoint> Velocity(this->NBreakpoint_);

                for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                {
                    APoint RandomPosition;
                    RandomPosition.X = static_cast<int>(GenerateRandom(this->LowerBound_.X, this->UpperBound_.Y));
                    RandomPosition.Y = static_cast<int>(GenerateRandom(this->LowerBound_.Y, this->UpperBound_.Y));

                    APoint RandomVelocity;
                    RandomVelocity = (this->LowerBound_ - RandomPosition) +
                                     (this->UpperBound_ - LowerBound_) * GenerateRandom(0.0f, 1.0f);

                    Position[BreakpointIndex] = RandomPosition;
                    Velocity[BreakpointIndex] = RandomVelocity;
                }

                CurrentPopulation->Position = Position;
                CurrentPopulation->Velocity = Velocity;

                std::vector<APoint> Waypoint;
                double FitnessValue = FitnessFunction(CurrentPopulation->Position, Start, Goal, Waypoint, CostMap);

                CurrentPopulation->FitnessValue = FitnessValue;

                this->AverageFitnessValue_ += FitnessValue;

                CurrentPopulation->BestPosition = CurrentPopulation->Position;
                CurrentPopulation->BestFitnessValue = FitnessValue;
                CurrentPopulation->Feedback = std::vector<APoint> (this->NBreakpoint_);

                if (FitnessValue < this->GlobalBestFitnessValue_)
                {
                    this->GlobalBestPosition_ = CurrentPopulation->Position;
                    this->GlobalBestFitnessValue_ = FitnessValue;
                }
            }

            this->AverageFitnessValue_ /= this->NPopulation_;

            // Optimize
            for (int Iteration = 1; Iteration <= this->MaximumIteration_; Iteration++)
            {
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    Optimize(Iteration, CurrentPopulation, Start, Goal, CostMap);
                }

                this->NextAverageFitnessValue_ /= this->NPopulation_;
                this->AverageFitnessValue_ = this->NextAverageFitnessValue_;
                this->NextAverageFitnessValue_ = 0.0f;

                if (this->Log_)
                {
                    std::cout << "[INFO] Iteration: " << Iteration << " >>> " << "Best fitness value: " << this->GlobalBestFitnessValue_ << std::endl;
                }
            }

            std::cout << "[INFO] Completed." << std::endl;

            Path.clear();

            Path.emplace_back(static_cast<int>(Start.X), static_cast<int>(Start.Y));

            for (const auto Waypoint : this->Waypoint_)
            {
                int X = static_cast<int>(Waypoint.X);
                int Y = static_cast<int>(Waypoint.Y);

                if (X != Path.back().X || Y != Path.back().Y)
                {
                    Path.emplace_back(X, Y);
                }
            }

            this->Path_ = Path;

            if (Path.empty())
            {
                std::cerr << "[INFO] Path not found!" << std::endl;

                return FAILED;
            }

            return SUCCESS;
        }

        std::vector<APoint> GetGlobalBestPosition () const
        {
            return this->GlobalBestPosition_;
        }

        double GetGlobalBestFitnessValue () const
        {
            return this->GlobalBestFitnessValue_;
        }

        std::vector<APoint> GetWaypoint () const
        {
            return this->Waypoint_;
        }

        double GetPathLength ()
        {
            double Length = 0.0f;

            for (int WaypointIndex = 0; WaypointIndex < this->Path_.size(); WaypointIndex++)
            {
                CalculateLength(Length, this->Path_, WaypointIndex);
            }

            return Length;
        }

    private:
        APoint LowerBound_, UpperBound_;
        int MaximumIteration_, NPopulation_, NBreakpoint_, NWaypoint_;
        double InertialWeight_{}, SocialCoefficient_, CognitiveCoefficient_;
        double MaximumInertialWeight_ = 0.9f, MinimumInertialWeight_ = 0.4;
        double VelocityFactor_;
        double ObstacleCostFactor_ = 1.00f;
        double FitnessValueScalingFactor_ = 1000.0f;
        double PenaltyScalingFactor_ = 100.0f;

        std::vector<AParticle> Population_;
        APoint MaximumVelocity_, MinimumVelocity_;

        APoint Range_;
        int N_{};

        std::vector<APoint> GlobalBestPosition_;
        double GlobalBestFitnessValue_ = (double)INFINITY;
        double AverageFitnessValue_ = 0.0f;
        double NextAverageFitnessValue_ = 0.0f;

        std::vector<APoint> Waypoint_;
        std::vector<APoint> Path_;

        PATH_TYPE PathType_;
        bool Log_;

        APoint UpdateVelocity (const AParticle *CurrentPopulation, int BreakpointIndex)
        {
            APoint NewVelocity;
            NewVelocity.X = this->InertialWeight_ * CurrentPopulation->Velocity[BreakpointIndex].X +
                            this->SocialCoefficient_ * GenerateRandom(0.0f, 1.0f) * (CurrentPopulation->BestPosition[BreakpointIndex].X - CurrentPopulation->Position[BreakpointIndex].X) +
                            this->CognitiveCoefficient_ * GenerateRandom(0.0f, 1.0f) * (this->GlobalBestPosition_[BreakpointIndex].X - CurrentPopulation->Position[BreakpointIndex].X) +
                            CurrentPopulation->Feedback[BreakpointIndex].X;
            NewVelocity.Y = this->InertialWeight_ * CurrentPopulation->Velocity[BreakpointIndex].Y +
                            this->SocialCoefficient_ * GenerateRandom(0.0f, 1.0f) * (CurrentPopulation->BestPosition[BreakpointIndex].Y - CurrentPopulation->Position[BreakpointIndex].Y) +
                            this->CognitiveCoefficient_ * GenerateRandom(0.0f, 1.0f) * (this->GlobalBestPosition_[BreakpointIndex].Y - CurrentPopulation->Position[BreakpointIndex].Y) +
                            CurrentPopulation->Feedback[BreakpointIndex].Y;

            NewVelocity.X = CLAMP(NewVelocity.X, this->MinimumVelocity_.X, this->MaximumVelocity_.X);
            NewVelocity.Y = CLAMP(NewVelocity.Y, this->MinimumVelocity_.Y, this->MaximumVelocity_.Y);

            return NewVelocity;
        }

        APoint UpdatePosition (AParticle *CurrentPopulation, int BreakpointIndex) const
        {
            APoint TemporaryNewPosition = CurrentPopulation->Position[BreakpointIndex] + CurrentPopulation->Velocity[BreakpointIndex];

            if (IS_OUT_OF_BOUND(TemporaryNewPosition.X, this->LowerBound_.X, this->UpperBound_.X) ||
                IS_OUT_OF_BOUND(TemporaryNewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y))
            {
                APoint VelocityConfinement = CurrentPopulation->Velocity[BreakpointIndex] * -GenerateRandom(0.0f, 1.0f);

                CurrentPopulation->Velocity[BreakpointIndex] = VelocityConfinement;
            }

            APoint NewPosition = CurrentPopulation->Position[BreakpointIndex] + CurrentPopulation->Velocity[BreakpointIndex];

            NewPosition.X = CLAMP(NewPosition.X, this->LowerBound_.X, this->UpperBound_.X);
            NewPosition.Y = CLAMP(NewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y);

            return NewPosition;
        }

        static void CalculateLength (double &Length, const std::vector<APoint> &Waypoint, int WaypointIndex)
        {
            if (WaypointIndex >= 1)
            {
                double DX = Waypoint[WaypointIndex].X - Waypoint[WaypointIndex - 1].X;
                double DY = Waypoint[WaypointIndex].Y - Waypoint[WaypointIndex - 1].Y;
                Length += std::hypot(DX, DY);
            }
        }

        int XYToIndex (int X, int Y) const
        {
            return Y * static_cast<int>(this->Range_.X) + X;
        }

        double Penalty (const unsigned char *CostMap, const std::vector<APoint> &Waypoint) const
        {
            int BreakpointIndex = 0;
            int NInterpolationPoint = (this->NWaypoint_ - 1) / (this->NBreakpoint_ + 1);
            double Penalty = 0.0f;

            for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; WaypointIndex++)
            {
                    int X = static_cast<int>(Waypoint[WaypointIndex].X);
                    int Y = static_cast<int>(Waypoint[WaypointIndex].Y);

                    int Index = XYToIndex(X, Y);

                    if (Index < 0 || Index >= this->N_ || CostMap[Index] >= this->ObstacleCostFactor_ * LETHAL_COST)
                    {
                        if (WaypointIndex == BreakpointIndex)
                        {
                            Penalty += 100.0f;
                        }
                        else
                        {
                            Penalty += 20.0;
                        }
                    }

                if (WaypointIndex % NInterpolationPoint == 0)
                {
                    BreakpointIndex += NInterpolationPoint;
                }
            }

            return Penalty;
        }

        void LinearPath (double &Length,
                         std::vector<APoint> &Waypoint,
                         const std::vector<double> &X,
                         const std::vector<double> &Y) const
        {
            int NInterpolationPoint = (this->NWaypoint_ - 1) / (this->NBreakpoint_ + 1);

            Waypoint.clear();
            Waypoint.emplace_back(X[0], Y[0]);

            for (int i = 1; i < this->NBreakpoint_ + 2; i++)
            {
                std::vector<double> InterpolationX = LinearInterpolation(X[i - 1], X[i], NInterpolationPoint);
                std::vector<double> InterpolationY = LinearInterpolation(Y[i - 1], Y[i], NInterpolationPoint);

                for (int j = 0; j < NInterpolationPoint; j++)
                {
                    Waypoint.emplace_back(InterpolationX[j], InterpolationY[j]);
                }
            }

            for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; WaypointIndex++)
            {
                CalculateLength(Length, Waypoint, WaypointIndex);
            }
        }

        void CubicSplinePath (double &Length,
                              std::vector<APoint> &Waypoint,
                              const std::vector<double> &X,
                              const std::vector<double> &Y) const
        {
            std::vector<double> Interpolation = LinearInterpolation(0.0f, 1.0f, this->NBreakpoint_ + 2);

            tk::spline CubicSplineX(Interpolation, X);
            tk::spline CubicSplineY(Interpolation, Y);

            Interpolation = LinearInterpolation(0.0f, 1.0f, this->NWaypoint_);

            Waypoint.clear();

            for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; WaypointIndex++)
            {
                double XInterpolation = CubicSplineX(Interpolation[WaypointIndex]);
                double YInterpolation = CubicSplineY(Interpolation[WaypointIndex]);

                Waypoint.emplace_back(XInterpolation, YInterpolation);

                CalculateLength(Length, Waypoint, WaypointIndex);
            }
        }

        double FitnessFunction (const std::vector<APoint> &Position,
                                const APoint &Start, const APoint &Goal,
                                std::vector<APoint> &Waypoint,
                                const unsigned char *CostMap) const
        {
            std::vector<double> X(this->NBreakpoint_ + 2);
            std::vector<double> Y(this->NBreakpoint_ + 2);

            X.front() = Start.X;
            Y.front() = Start.Y;

            for (int BreakpointIndex = 1; BreakpointIndex <= this->NBreakpoint_; BreakpointIndex++)
            {
                X[BreakpointIndex] = Position[BreakpointIndex - 1].X;
                Y[BreakpointIndex] = Position[BreakpointIndex - 1].Y;
            }

            X.back() = Goal.X;
            Y.back() = Goal.Y;

            double Length = 0.0f;

            switch (this->PathType_)
            {
                case LINEAR:
                    LinearPath(Length, Waypoint, X, Y);
                    break;
                case CUBIC_SPLINE:
                    CubicSplinePath(Length, Waypoint, X, Y);
                    break;
                default:
                    CubicSplinePath(Length, Waypoint, X, Y);
            }

            double Error = Penalty(CostMap, Waypoint);
            Error = (Error == 0.0f ? 0.0f : this->PenaltyScalingFactor_ * Error);

            double FitnessValue = this->FitnessValueScalingFactor_ * Length * (1.0f + Error);

            return FitnessValue;
        }

        void CalculateAdaptiveInertialWeight (AParticle *CurrentPopulation)
        {
            if (CurrentPopulation->FitnessValue <= this->AverageFitnessValue_)
            {
                this->InertialWeight_ = this->MinimumInertialWeight_ + (this->MaximumInertialWeight_ - this->MinimumInertialWeight_) *
                                                                       ((CurrentPopulation->FitnessValue - this->GlobalBestFitnessValue_) / (this->AverageFitnessValue_ - this->GlobalBestFitnessValue_));
            }
            else
            {
                this->InertialWeight_ = this->MinimumInertialWeight_ + (this->MaximumInertialWeight_ - this->MinimumInertialWeight_) *
                                                                       ((this->AverageFitnessValue_ - this->GlobalBestFitnessValue_) / (CurrentPopulation->FitnessValue - this->GlobalBestFitnessValue_));
            }
        }

        void Optimize (int Iteration, AParticle *CurrentPopulation,
                       const APoint &Start, const APoint &Goal,
                       const unsigned char *CostMap)
        {
            CalculateAdaptiveInertialWeight(CurrentPopulation);

            // Update Velocity
            for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
            {
                APoint NewVelocity = UpdateVelocity(CurrentPopulation, BreakpointIndex);

                CurrentPopulation->Velocity[BreakpointIndex] = NewVelocity;
            }

            // Update Position
            for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
            {
                APoint NewPosition = UpdatePosition(CurrentPopulation, BreakpointIndex);

                CurrentPopulation->Position[BreakpointIndex] = NewPosition;
            }

            // Evaluate Fitness Value
            std::vector<APoint> Waypoint;
            double FitnessValue = FitnessFunction(CurrentPopulation->Position, Start, Goal, Waypoint, CostMap);
            CurrentPopulation->FitnessValue = FitnessValue;

            this->NextAverageFitnessValue_ += FitnessValue;

            // Update PBest
            if (FitnessValue < CurrentPopulation->BestFitnessValue)
            {
                CurrentPopulation->BestPosition = CurrentPopulation->Position;
                CurrentPopulation->BestFitnessValue = FitnessValue;

                for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                {
                    CurrentPopulation->Feedback[BreakpointIndex] = CurrentPopulation->Feedback[BreakpointIndex] * (1.0f / static_cast<double>(Iteration)) +
                                                                   (CurrentPopulation->Velocity[BreakpointIndex]) *
                                                                   GenerateRandom(0.0f, 1.0f);
                }
            }
            else
            {
                for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                {
                    CurrentPopulation->Feedback[BreakpointIndex] = CurrentPopulation->Feedback[BreakpointIndex] * (1.0f / static_cast<double>(Iteration)) -
                                                                   (CurrentPopulation->Velocity[BreakpointIndex]) *
                                                                   GenerateRandom(0.0f, 1.0f);
                }
            }

            // Update GBest
            if (FitnessValue < this->GlobalBestFitnessValue_)
            {
                this->GlobalBestPosition_ = CurrentPopulation->Position;
                this->GlobalBestFitnessValue_ = FitnessValue;

                this->Waypoint_ = Waypoint;
            }
        }
    };
} // Optimizer

#endif // PLANNER_H
