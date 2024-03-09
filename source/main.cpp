//
// Created by Sippawit Thammawiset on 3/2/2024 AD.
//

#include "Print.h"
#include "CostMapLoader.h"
#include "ABCPlanner.h"
#include "PSOPlanner.h"
#include "GWOPlanner.h"

int main ()
{
    int Width, Height;
    MTH::APoint Start, Goal;
    int CostMapName = CostMapLoader::MAP::SCENARIO_2;

    // Load cost map
    auto CostMap = CostMapLoader::CostMapLoader(Width, Height, CostMapName,
                                                              Start, Goal);
    // Global parameter configuration
    int MaximumIteration = 100;
    int NPopulation = 50;
    int NBreakpoint = 3;
    int NInterpolationPoint = 30;
    int NWaypoint = 1 + (NBreakpoint + 1) * NInterpolationPoint;
    double MaximumWeight = 0.9f;
    double MinimumWeight = 0.4f;
    double VelocityFactor = 0.5;
    MTH::INITIAL_POSITION_TYPE InitialPositionType = MTH::INITIAL_POSITION::CIRCULAR;
    MTH::TRAJECTORY_TYPE TrajectoryType = MTH::TRAJECTORY::CUBIC_SPLINE;
    bool Log = true;

    MTH::APoint LowerBound(0.0f, 0.0f);
    MTH::APoint UpperBound(Width, Height);

    // PSO parameter configuration
    double SocialCoefficient = 2.0f;
    double CognitiveCoefficient = 1.3f;
    int VelocityConfinement = MTH::PSO::VELOCITY_CONFINEMENT::HYPERBOLIC;

    // GWO parameter configuration
    double Theta = 2.2f;
    double K = 1.5f;
    double Maximum_a = 2.2f;
    double Minimum_a = 0.02f;

    int NRun = 1;

    MTH::PSO::APSOPlanner PSOPlanner(LowerBound, UpperBound,
                                     MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                     SocialCoefficient, CognitiveCoefficient,
                                     MaximumWeight, MinimumWeight,
                                     VelocityConfinement,
                                     VelocityFactor,
                                     InitialPositionType,
                                     TrajectoryType,
                                     Log);

    MTH::GWO::AGWOPlanner GWOPlanner(LowerBound, UpperBound,
                                     MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                     Theta, K,
                                     Maximum_a, Minimum_a,
                                     MaximumWeight, MinimumWeight,
                                     VelocityFactor,
                                     InitialPositionType,
                                     TrajectoryType,
                                     Log);

    MTH::ABC::AABCPlanner ABCPlanner(LowerBound, UpperBound,
                                     MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                     InitialPositionType,
                                     TrajectoryType,
                                     Log);

    MTH::ABasePlanner *Planner = &ABCPlanner;

    for (int Run = 1; Run <= NRun; Run++)
    {
        std::vector<MTH::APoint> Waypoint;

        auto StartTime = std::chrono::high_resolution_clock::now();
        if (Planner->CreatePlan(CostMap, Start, Goal, Waypoint))
        {
            auto StopTime = std::chrono::high_resolution_clock::now();
            auto Duration = std::chrono::duration_cast<std::chrono::milliseconds>(StopTime - StartTime);

            std::cout << "Breakpoint: " << Planner->GetGlobalBestPosition() << std::endl;
            std::cout << "Waypoint: " << Waypoint << std::endl;
            std::cout << "Length (px): " << Planner->GetPathLength() << std::endl;
            std::cout << "Execution time (ms): " << Duration.count() << " milliseconds" << std::endl;
        }

        Planner->Clear();
    }

    return 0;
}

