//
// Created by Sippawit Thammawiset on 13/1/2024 AD.
//

#include "PSO.h"
#include "spline.h"

#include <iostream>
#include <random>
#include <thread>

#define CLAMP(X, MIN, MAX)          std::max(MIN, std::min(MAX, X))

namespace Optimizer
{
    APSO::APSO (APoint LowerBound,
                APoint UpperBound,
                int MaxIteration, int Population, int Breakpoint, int Waypoint,
                double InertialCoefficient, double SocialCoefficient, double CognitiveCoefficient,
                double VelocityFactor, double ObstacleCostFactor,
                double FitnessValueScalingFactor, double PenaltyScalingFactor) :
                LowerBound_(LowerBound),
                UpperBound_(UpperBound),
                MaxIteration_(MaxIteration),
                NPopulation_(Population),
                NBreakpoint_(Breakpoint),
                NWaypoint_(Waypoint),
                InertialCoefficient_(InertialCoefficient),
                SocialCoefficient_(SocialCoefficient),
                CognitiveCoefficient_(CognitiveCoefficient),
                VelocityFactor_(VelocityFactor),
                ObstacleCostFactor_(ObstacleCostFactor),
                FitnessValueScalingFactor_(FitnessValueScalingFactor),
                PenaltyScalingFactor_(PenaltyScalingFactor)
    {
        this->Population_ = std::vector<AParticle> (NPopulation_);

        this->Range_ = this->UpperBound_ - this->LowerBound_;
        this->N_ = static_cast<int>(this->Range_.X * this->Range_.Y);

        this->MaximumVelocity_ = this->Range_ * this->VelocityFactor_;
        this->MinimumVelocity_ = MaximumVelocity_ * -1.0f;
    }

    APSO::~APSO () = default;

    APoint APSO::InitializePosition () const
    {
        APoint RandomPosition;
        RandomPosition.X = static_cast<int>(GenerateRandom(this->LowerBound_.X, this->UpperBound_.Y));
        RandomPosition.Y = static_cast<int>(GenerateRandom(this->LowerBound_.Y, this->UpperBound_.Y));

        return RandomPosition;
    }

    APoint APSO::InitializeVelocity ()
    {
        APoint RandomVelocity;
        RandomVelocity.X = GenerateRandom(0.0f, 1.0f);
        RandomVelocity.Y = GenerateRandom(0.0f, 1.0f);

        return RandomVelocity;
    }

    void APSO::CalculateLength (double &Length, const std::vector<APoint> &Waypoint, int WaypointIndex)
    {
        if (WaypointIndex >= 1)
        {
            double DX = Waypoint[WaypointIndex].X - Waypoint[WaypointIndex - 1].X;
            double DY = Waypoint[WaypointIndex].Y - Waypoint[WaypointIndex - 1].Y;
            Length += sqrt(DX * DX + DY * DY);
        }
    }

    int APSO::XYToIndex (int X, int Y) const
    {
        return Y * static_cast<int>(this->Range_.X) + X;
    }

    double APSO::Penalty (const unsigned char *Costmap, const std::vector<APoint> &Waypoint) const
    {
        double Penalty = 0.0f;

        for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; WaypointIndex++)
        {
            int Index = XYToIndex(static_cast<int>(Waypoint[WaypointIndex].X),
                                  static_cast<int>(Waypoint[WaypointIndex].Y));

            if (Index < 0 || Index >= this->N_ || Costmap[Index] >= this->ObstacleCostFactor_ * LETHAL_COST)
            {
                Penalty++;
            }
        }

        return Penalty;
    }

    double APSO::FitnessFunction (AParticle *Population,
                                  const APoint &Start, const APoint &Goal,
                                  std::vector<APoint> &Waypoint,
                                  const unsigned char *Costmap) const
    {
        std::vector<double> X(this->NBreakpoint_ + 2);
        std::vector<double> Y(this->NBreakpoint_ + 2);

        X.front() = Start.X;
        Y.front() = Start.Y;

        for (int BreakpointIndex = 1; BreakpointIndex <= this->NBreakpoint_; BreakpointIndex++)
        {
            X[BreakpointIndex] = Population->Position[BreakpointIndex - 1].X;
            Y[BreakpointIndex] = Population->Position[BreakpointIndex - 1].Y;
        }

        X.back() = Goal.X;
        Y.back() = Goal.Y;

        std::vector<double> Interpolation = LinearInterpolation(0.0f, 1.0f, this->NBreakpoint_ + 2);

        tk::spline CubicSplineX(Interpolation, X);
        tk::spline CubicSplineY(Interpolation, Y);

        Interpolation = LinearInterpolation(0.0f, 1.0f, this->NWaypoint_);
        Waypoint = std::vector<APoint> (this->NWaypoint_);

        double Length = 0.0f;

        for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; WaypointIndex++)
        {
            Waypoint[WaypointIndex] = APoint(CubicSplineX(Interpolation[WaypointIndex]), CubicSplineY(Interpolation[WaypointIndex]));

            CalculateLength(Length, Waypoint, WaypointIndex);
        }

        double Error = Penalty(Costmap, Waypoint);
        Error = (Error == 0.0f ? 0.0f : this->PenaltyScalingFactor_ / Error);

//        double FitnessValue = Length * (1.0f + Error);
        double FitnessValue = this->FitnessValueScalingFactor_ / (Length * (1.0f + Error));

        return FitnessValue;
    }

    APoint APSO::UpdateVelocity (AParticle *Population, int BreakpointIndex) const
    {
        APoint NewVelocity;
        NewVelocity.X =
                this->InertialCoefficient_ * Population->Velocity[BreakpointIndex].X +
                this->SocialCoefficient_ * GenerateRandom(0.0f, 1.0f) * (Population->BestPosition[BreakpointIndex].X - Population->Position[BreakpointIndex].X) +
                this->CognitiveCoefficient_ * GenerateRandom(0.0f, 1.0f) * (this->GlobalBestPosition_[BreakpointIndex].X - Population->Position[BreakpointIndex].X);
        NewVelocity.Y =
                this->InertialCoefficient_ * Population->Velocity[BreakpointIndex].Y +
                this->SocialCoefficient_ * GenerateRandom(0.0f, 1.0f) * (Population->BestPosition[BreakpointIndex].Y - Population->Position[BreakpointIndex].Y) +
                this->CognitiveCoefficient_ * GenerateRandom(0.0f, 1.0f) * (this->GlobalBestPosition_[BreakpointIndex].Y - Population->Position[BreakpointIndex].Y);

        NewVelocity.X = CLAMP(NewVelocity.X, this->MinimumVelocity_.X, this->MaximumVelocity_.X);
        NewVelocity.Y = CLAMP(NewVelocity.Y, this->MinimumVelocity_.Y, this->MaximumVelocity_.Y);

        return NewVelocity;
    }

    APoint APSO::UpdatePosition (AParticle *Population, int BreakpointIndex) const
    {
        APoint NewPosition;
        NewPosition = Population->Position[BreakpointIndex] + Population->Velocity[BreakpointIndex];

        // TODO: Improve this!
        NewPosition.X = CLAMP(NewPosition.X, this->LowerBound_.X, this->UpperBound_.X);
        NewPosition.Y = CLAMP(NewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y);

        return NewPosition;
    }

    bool APSO::CreatePlan (const unsigned char *Costmap,
                           const APoint &Start,
                           const APoint &Goal,
                           std::vector<APoint> &Path)
    {
        // Initialize
        for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
        {
            AParticle *CurrentPopulation = &this->Population_[PopulationIndex];

            std::vector<APoint> Position(this->NBreakpoint_);
            std::vector<APoint> Velocity(this->NBreakpoint_);

            for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
            {
                Position[BreakpointIndex] = InitializePosition();
                Velocity[BreakpointIndex] = InitializeVelocity();
            }

            CurrentPopulation->Position = Position;
            CurrentPopulation->Velocity = Velocity;

            std::vector<APoint> Waypoint;
            double FitnessValue = FitnessFunction(CurrentPopulation, Start, Goal, Waypoint, Costmap);

            CurrentPopulation->BestPosition = CurrentPopulation->Position;
            CurrentPopulation->BestFitnessValue = FitnessValue;

            if (FitnessValue > this->GlobalBestFitnessValue_)
            {
                this->GlobalBestPosition_ = CurrentPopulation->Position;
                this->GlobalBestFitnessValue_ = FitnessValue;
            }
        }

        // Optimize
        for (int Iteration = 0; Iteration < this->MaxIteration_; Iteration++)
        {
            std::vector<std::thread> PopulationList = std::vector<std::thread> (this->NPopulation_);

            for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
            {
                AParticle *CurrentPopulation = &this->Population_[PopulationIndex];

                PopulationList[PopulationIndex] = std::thread(&APSO::Optimize, this, CurrentPopulation, Start, Goal, Costmap);
            }

            for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
            {
                PopulationList[PopulationIndex].join();
            }

            std::cout << "[INFO] Iteration: " << Iteration << " >>> " << "Best fitness value: " << this->GlobalBestFitnessValue_ << std::endl;
        }

        std::cout << "[INFO] Completed, Optimal fitness value: " << this->GlobalBestFitnessValue_ << std::endl;

        Path.clear();

        if (!this->Waypoint_.empty())
        {
            Path.emplace_back(static_cast<int>(this->Waypoint_.front().X),
                              static_cast<int>(this->Waypoint_.front().Y));

            for (const auto &Waypoint : this->Waypoint_)
            {
                int X = static_cast<int>(Waypoint.X);
                int Y = static_cast<int>(Waypoint.Y);

                if (X != Path.back().X || Y != Path.back().Y)
                {
                    Path.emplace_back(X, Y);
                }
            }
        }

        if (!Path.empty())
        {
            std::cout << "[INFO] Successful planning" << std::endl;
        }

        return !Path.empty();
    }

    void APSO::Optimize (AParticle *CurrentPopulation,
                         const APoint &Start, const APoint &Goal,
                         const unsigned char *Costmap)
    {
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
        double FitnessValue = FitnessFunction(CurrentPopulation, Start, Goal, Waypoint, Costmap);

        // Update PBest
        if (FitnessValue > CurrentPopulation->BestFitnessValue)
        {
            CurrentPopulation->BestPosition = CurrentPopulation->Position;
            CurrentPopulation->BestFitnessValue = FitnessValue;
        }

        GlobalBestLock.lock();

        // Update GBest
        if (FitnessValue > this->GlobalBestFitnessValue_)
        {
            this->GlobalBestPosition_ = CurrentPopulation->Position;
            this->GlobalBestFitnessValue_ = FitnessValue;

            this->Waypoint_ = Waypoint;
        }

        GlobalBestLock.unlock();
    }

    double GenerateRandom (double LowerBound, double UpperBound)
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
}
