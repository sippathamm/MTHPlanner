//
// Created by Sippawit Thammawiset on 15/1/2024 AD.
//

#include "PSO.h"

#include <iostream>
#include <random>
#include <utility>

#define CLAMP(X, MIN, MAX)        std::max(MIN, std::min(MAX, X))

typedef struct AParticle
{
    AParticle () : BestFitnessValue(INFINITY) {}

    std::vector<double> Position;
    std::vector<double> Velocity;

    std::vector<double> BestPosition;
    double BestFitnessValue;
} AParticle;

double GenerateRandom (double LowerBound = 0.0f, double UpperBound = 1.0f)
{
    std::random_device Engine;
    std::uniform_real_distribution<double> RandomDistribution(0.0f, 1.0f);
    return LowerBound + RandomDistribution(Engine) * (UpperBound - LowerBound);
}

double FitnessFunction (const std::vector<double> &Position)
{
    double X0 = Position[0];
    double X1 = Position[1];

    return pow(X0 - 3.14f, 2) + pow(X1 - 2.72f, 2) + sin(3 * X0 + 1.41f) + sin(4 * X1 - 1.73f);
}

double UpdateVelocity (const AParticle *CurrentPopulation, int VariableIndex, const std::vector<double> &GlobalBestPosition,
                       double InertialCoefficient, double SocialCoefficient, double CognitiveCoefficient,
                       const std::vector<double> &MinimumVelocity, const std::vector<double> &MaximumVelocity)
{
    double NewVelocity =
            InertialCoefficient * CurrentPopulation->Velocity[VariableIndex] +
            SocialCoefficient * GenerateRandom(0.0f, 1.0f) * (CurrentPopulation->BestPosition[VariableIndex] - CurrentPopulation->Position[VariableIndex])+
            CognitiveCoefficient * GenerateRandom(0.0f, 1.0f) * (GlobalBestPosition[VariableIndex] - CurrentPopulation->Position[VariableIndex]);

    NewVelocity = CLAMP(NewVelocity, MinimumVelocity[VariableIndex], MaximumVelocity[VariableIndex]);

    return NewVelocity;
}

double UpdatePosition (const AParticle *CurrentPopulation, int VariableIndex,
                       double NewVelocity,
                       const std::vector<double> &LowerBound, const std::vector<double> &UpperBound)
{
    double NewPosition = CurrentPopulation->Position[VariableIndex] + NewVelocity;

    NewPosition = CLAMP(NewPosition, LowerBound[VariableIndex], UpperBound[VariableIndex]);

    return NewPosition;
}

int main()
{
    int MaxIteration = 500;
    int NPopulation = 50;
    int NVariable = 2;
    float VelocityFactor = 0.5f;
    double InertialCoefficient = 0.9f;
    double SocialCoefficient = 1.5f;
    double CognitiveCoefficient = 1.5f;

    std::vector<double> LowerBound({0, 0});
    std::vector<double> UpperBound({5, 5});

    std::vector<double> MaximumVelocity(NVariable);
    std::vector<double> MinimumVelocity(NVariable);

    for (int VariableIndex = 0; VariableIndex < NVariable; VariableIndex++)
    {
        MaximumVelocity[VariableIndex] = VelocityFactor * (UpperBound[VariableIndex] - LowerBound[VariableIndex]);
        MinimumVelocity[VariableIndex] = -MaximumVelocity[VariableIndex];
    }

    std::vector<AParticle> Population(NPopulation);

    std::vector<double> GlobalBestPosition;
    double GlobalBestFitnessValue = INFINITY;

    // Initializing
    for (int PopulationIndex = 0; PopulationIndex < NPopulation; PopulationIndex++)
    {
        std::vector<double> Position(NVariable);
        std::vector<double> Velocity(NVariable);

        for (int VariableIndex = 0; VariableIndex < NVariable; VariableIndex++)
        {
            double RandomPosition = GenerateRandom(LowerBound[VariableIndex], UpperBound[VariableIndex]);
            double RandomVelocity = GenerateRandom(0.0f, 1.0f);

            Position[VariableIndex] = RandomPosition;
            Velocity[VariableIndex] = RandomVelocity;
        }

        Population[PopulationIndex].Position = Position;
        Population[PopulationIndex].Velocity = Velocity;
    }

    // Evaluate Fitness Value
    for (int PopulationIndex = 0; PopulationIndex < NPopulation; PopulationIndex++)
    {
        AParticle *CurrentPopulation = &Population[PopulationIndex];

        double FitnessValue = FitnessFunction(CurrentPopulation->Position);

        CurrentPopulation->BestPosition = CurrentPopulation->Position;
        CurrentPopulation->BestFitnessValue = FitnessValue;

        if (FitnessValue < GlobalBestFitnessValue)
        {
            GlobalBestPosition = CurrentPopulation->Position;
            GlobalBestFitnessValue = FitnessValue;
        }
    }

    // Optimizing
    for (int Iteration = 0; Iteration < MaxIteration; Iteration++)
    {
        for (int PopulationIndex = 0; PopulationIndex < NPopulation; PopulationIndex++)
        {
            AParticle *CurrentPopulation = &Population[PopulationIndex];

            std::vector<double> UpdatedPosition(NVariable);
            std::vector<double> UpdatedVelocity(NVariable);

            for (int VariableIndex = 0; VariableIndex < NVariable; VariableIndex++)
            {
                // Update Velocity
                double NewVelocity = UpdateVelocity(CurrentPopulation, VariableIndex, GlobalBestPosition,
                                                    InertialCoefficient, SocialCoefficient, CognitiveCoefficient,
                                                    MinimumVelocity, MaximumVelocity);
                // Update Position
                double NewPosition = UpdatePosition(CurrentPopulation, VariableIndex, NewVelocity, LowerBound, UpperBound);

                UpdatedPosition[VariableIndex] = NewPosition;
                UpdatedVelocity[VariableIndex] = NewVelocity;
            }

            // Evaluate Fitness Value
            double FitnessValue = FitnessFunction(UpdatedPosition);

            // Update PBest
            if (FitnessValue < CurrentPopulation->BestFitnessValue)
            {
                CurrentPopulation->BestPosition = UpdatedPosition;
                CurrentPopulation->BestFitnessValue = FitnessValue;
            }

            // Update GBest
            if (FitnessValue < GlobalBestFitnessValue)
            {
                GlobalBestPosition = UpdatedPosition;
                GlobalBestFitnessValue = FitnessValue;
            }

            CurrentPopulation->Position = UpdatedPosition;
            CurrentPopulation->Velocity = UpdatedVelocity;
        }

        std::cout << "Iteration: " << Iteration << " >> " << \
        "Best Global Fitness Value: " << GlobalBestFitnessValue << "\n";
    }

    std::cout << "Best Global Position: ";

    for (auto i : GlobalBestPosition)
    {
        std::cout << i << " ";
    }

    std::cout << "\n";
    std::cout << "Best Global Fitness Value: " << GlobalBestFitnessValue << "\n";

    return 0;
}
