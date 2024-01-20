//
// Created by Sippawit Thammawiset on 15/1/2024 AD.
//

#include "PSO.h"
#include "spline.h"

#include <iostream>
#include <random>

#define CLAMP(X, MIN, MAX)        std::max(MIN, std::min(MAX, X))

PSO_Planner::APoint LowerBound(-20, -20);
PSO_Planner::APoint UpperBound(20, 20);

PSO_Planner::APoint Start(0, 5);
PSO_Planner::APoint Goal(8, -2);

int MaxIteration = 2500;
int NPopulation = 50;
int NBreakpoint = 3;
int NInterpolationPoint = 50;
int NWaypoint = 1 + (NBreakpoint + 1) * NInterpolationPoint;

double Generator ()
{
    std::random_device Engine;
    std::uniform_real_distribution<double> RandomDistribution(0.0f, 1.0f);
    return RandomDistribution(Engine);
}

double GenerateRandom (double Minimum, double Maximum)
{
    return Minimum + Generator() * (Maximum - Minimum);
}

std::vector<double> LinearInterpolation (double Begin, double End, int Size)
{
    std::vector<double> Interpolation;

    if (Size == 0)
    {
        return Interpolation;
    }

    if (Size == 1)
    {
        Interpolation.push_back(Begin);
        return Interpolation;
    }

    double Delta = (End - Begin) / (Size - 1);

    for (int i = 0; i < Size - 1; i++)
    {
        Interpolation.push_back(Begin + Delta * i);
    }

    Interpolation.push_back(End);

    return Interpolation;
}

int main()
{
    PSO_Planner::APoint MaximumVelocity = (UpperBound - LowerBound) * 0.5f;
    PSO_Planner::APoint MinimumVelocity = MaximumVelocity * -1.0f;

    std::vector<PSO_Planner::AParticle> Population(NPopulation);

    std::vector<PSO_Planner::APoint> GlobalBestPosition;
    double GlobalBestFitnessValue = INFINITY;
    std::vector<PSO_Planner::APoint> BestWaypoint;

    // Initializing
    for (int PopulationIndex = 0; PopulationIndex < NPopulation; PopulationIndex++)
    {
        std::vector<PSO_Planner::APoint> Position(NBreakpoint);
        std::vector<PSO_Planner::APoint> Velocity(NBreakpoint);

        for (int BreakpointIndex = 0; BreakpointIndex < NBreakpoint; BreakpointIndex++)
        {
            PSO_Planner::APoint RandomPosition;
            RandomPosition.X = GenerateRandom(LowerBound.X, UpperBound.X);
            RandomPosition.Y = GenerateRandom(LowerBound.Y, UpperBound.Y);

            PSO_Planner::APoint RandomVelocity;
            RandomVelocity.X = GenerateRandom(0.0f, 1.0f);
            RandomVelocity.Y = GenerateRandom(0.0f, 1.0f);

            Position[BreakpointIndex] = RandomPosition;
            Velocity[BreakpointIndex] = RandomVelocity;
        }

        Population[PopulationIndex].Position = Position;
        Population[PopulationIndex].Velocity = Velocity;
    }

    // Evaluate Fitness Value
    for (int PopulationIndex = 0; PopulationIndex < NPopulation; PopulationIndex++)
    {
        std::vector<double> X(NBreakpoint + 2);
        std::vector<double> Y(NBreakpoint + 2);

        X.front() = Start.X;
        Y.front() = Start.Y;

        for (int BreakpointIndex = 1; BreakpointIndex <= NBreakpoint; BreakpointIndex++)
        {
            X[BreakpointIndex] = Population[PopulationIndex].Position[BreakpointIndex - 1].X;
            Y[BreakpointIndex] = Population[PopulationIndex].Position[BreakpointIndex - 1].Y;
        }

        X.back() = Goal.X;
        Y.back() = Goal.Y;

        std::vector<double> Interpolation = LinearInterpolation(0.0f, 1.0f, NBreakpoint + 2);

        tk::spline CubicSplineX(Interpolation, X);
        tk::spline CubicSplineY(Interpolation, Y);

        Interpolation = LinearInterpolation(0.0f, 1.0f, NWaypoint);
        std::vector<PSO_Planner::APoint> Waypoint(NWaypoint);

        double Length = 0.0f;

        for (int i = 0; i < NWaypoint; i++)
        {
            Waypoint[i] = PSO_Planner::APoint(CubicSplineX(Interpolation[i]), CubicSplineY(Interpolation[i]));

            if (i >= 1)
            {
                double DX = Waypoint[i].X - Waypoint[i - 1].X;
                double DY = Waypoint[i].Y - Waypoint[i - 1].Y;
                Length += sqrt(DX * DX + DY * DY);
            }
        }

        double FitnessValue = Length;

        Population[PopulationIndex].BestPosition = Population[PopulationIndex].Position;
        Population[PopulationIndex].BestFitnessValue = FitnessValue;

        if (FitnessValue < GlobalBestFitnessValue)
        {
            GlobalBestPosition = Population[PopulationIndex].Position;
            GlobalBestFitnessValue = FitnessValue;
        }
    }

    for (int Iteration = 0; Iteration < MaxIteration; Iteration++)
    {
        for (int PopulationIndex = 0; PopulationIndex < NPopulation; PopulationIndex++)
        {
            // Update Velocity
            for (int BreakpointIndex = 0; BreakpointIndex < NBreakpoint; BreakpointIndex++)
            {
                PSO_Planner::APoint NewVelocity;
                NewVelocity.X =
                        0.9 * Population[PopulationIndex].Velocity[BreakpointIndex].X +
                        1.5 * GenerateRandom(0.0f, 1.0f) * (Population[PopulationIndex].BestPosition[BreakpointIndex].X - Population[PopulationIndex].Position[BreakpointIndex].X) +
                        1.5 * GenerateRandom(0.0f, 1.0f) * (GlobalBestPosition[BreakpointIndex].X - Population[PopulationIndex].Position[BreakpointIndex].X);
                NewVelocity.Y =
                        0.9 * Population[PopulationIndex].Velocity[BreakpointIndex].Y +
                        1.5 * GenerateRandom(0.0f, 1.0f) * (Population[PopulationIndex].BestPosition[BreakpointIndex].Y - Population[PopulationIndex].Position[BreakpointIndex].Y) +
                        1.5 * GenerateRandom(0.0f, 1.0f) * (GlobalBestPosition[BreakpointIndex].Y - Population[PopulationIndex].Position[BreakpointIndex].Y);

                NewVelocity.X = CLAMP(NewVelocity.X, MinimumVelocity.X, MaximumVelocity.X);
                NewVelocity.Y = CLAMP(NewVelocity.Y, MinimumVelocity.Y, MaximumVelocity.Y);

                Population[PopulationIndex].Velocity[BreakpointIndex] = NewVelocity;
            }

            // Update Position
            for (int BreakpointIndex = 0; BreakpointIndex < NBreakpoint; BreakpointIndex++)
            {
                PSO_Planner::APoint NewPosition;
                NewPosition = Population[PopulationIndex].Position[BreakpointIndex] + Population[PopulationIndex].Velocity[BreakpointIndex];

                NewPosition.X = CLAMP(NewPosition.X, LowerBound.X, UpperBound.X);
                NewPosition.Y = CLAMP(NewPosition.Y, LowerBound.Y, UpperBound.Y);

                Population[PopulationIndex].Position[BreakpointIndex] = NewPosition;
            }

            // Evaluate Fitness Value
            std::vector<double> X(NBreakpoint + 2);
            std::vector<double> Y(NBreakpoint + 2);

            X.front() = Start.X;
            Y.front() = Start.Y;

            for (int BreakpointIndex = 1; BreakpointIndex <= NBreakpoint; BreakpointIndex++)
            {
                X[BreakpointIndex] = Population[PopulationIndex].Position[BreakpointIndex - 1].X;
                Y[BreakpointIndex] = Population[PopulationIndex].Position[BreakpointIndex - 1].Y;
            }

            X.back() = Goal.X;
            Y.back() = Goal.Y;

            std::vector<double> Interpolation = LinearInterpolation(0.0f, 1.0f, NBreakpoint + 2);

            tk::spline CubicSplineX(Interpolation, X);
            tk::spline CubicSplineY(Interpolation, Y);

            Interpolation = LinearInterpolation(0.0f, 1.0f, NWaypoint);
            std::vector<PSO_Planner::APoint> Waypoint(NWaypoint);

            double Length = 0.0f;

            for (int i = 0; i < NWaypoint; i++)
            {
                Waypoint[i] = PSO_Planner::APoint(CubicSplineX(Interpolation[i]), CubicSplineY(Interpolation[i]));

                if (i >= 1)
                {
                    double DX = Waypoint[i].X - Waypoint[i - 1].X;
                    double DY = Waypoint[i].Y - Waypoint[i - 1].Y;
                    Length += sqrt(DX * DX + DY * DY);
                }
            }

            double FitnessValue = Length;

            if (FitnessValue < Population[PopulationIndex].BestFitnessValue)
            {
                Population[PopulationIndex].BestPosition = Population[PopulationIndex].Position;
                Population[PopulationIndex].BestFitnessValue = FitnessValue;
            }

            if (FitnessValue < GlobalBestFitnessValue)
            {
                GlobalBestPosition = Population[PopulationIndex].Position;
                GlobalBestFitnessValue = FitnessValue;

                BestWaypoint = Waypoint;
            }
        }

        std::cout << "Iteration: " << Iteration << " | " << \
        "Best Global Fitness Value: " << GlobalBestFitnessValue << "\n";
    }

    for (auto i : BestWaypoint)
    {
        std::cout << i.X << "\t" << i.Y << "\n";
    }

    return 0;
}
