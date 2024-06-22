//
// Created by Sippawit Thammawiset on 3/2/2024 AD.
// TODO: - Separate .h and .cpp files
//

#include "Configuration.h"
#include "CostMapLoader.h"
#include "Print.h"
#include "ABCPlanner.h"
#include "IPSOPlanner.h"
#include "IGWOPlanner.h"

int main ()
{
    int Width; // Variable to store width of the cost map
    int Height; // Variable to store height of the cost map
    MTH::APoint Start; // Start point
    MTH::APoint Goal; // Goal point
    int CostMapName = CostMapLoader::MAP::TURTLEBOT3_WORLD; // Specify the desired cost map name

    // Load cost map and initialize start and goal points
    auto CostMap = CostMapLoader::CostMapLoader(CostMapName, Width, Height, Start, Goal);

    MTH::CONFIGURATION::APlannerConfiguration PlannerConfiguration;
    PlannerConfiguration.LowerBound = MTH::APoint (0, 0);
    PlannerConfiguration.UpperBound = MTH::APoint (Width, Height);
    PlannerConfiguration.MaximumIteration = 100;
    PlannerConfiguration.NPopulation = 50;
    PlannerConfiguration.InitialPositionType = MTH::INITIAL_POSITION::CIRCULAR;
    PlannerConfiguration.Log = true;

    MTH::CONFIGURATION::APathConfiguration PathConfiguration;
    PathConfiguration.NBreakpoint = 3;
    int NInterpolationPoint = 5;
    PathConfiguration.NWaypoint = 1 + (PathConfiguration.NBreakpoint + 1) * NInterpolationPoint;
    PathConfiguration.TrajectoryType = MTH::TRAJECTORY::CUBIC_SPLINE;

    double MaximumInertialWeight = 0.9; // Maximum inertial weight
    double MinimumInertialWeight = 0.4; // Minimum inertial weight
    double VelocityFactor = 0.5; // Velocity factor for limiting velocity update

    // IPSO parameters
    double SocialCoefficient = 2.0; // Social coefficient
    double CognitiveCoefficient = 1.3; // Cognitive coefficient
    int VelocityConfinement = MTH::IPSO::VELOCITY_CONFINEMENT::HYPERBOLIC; // Velocity confinement type

    // IGWO parameters
    double MaximumWeight = 2.2; // Maximum weight
    double MinimumWeight = 0.02; // Minimum weight

    // ABC parameters
    // No parameters

    int NRun = 1; // Number of runs

    // Initialize planners
    MTH::IPSO::AIPSOPlanner IPSOPlanner(PlannerConfiguration, PathConfiguration,
                                        SocialCoefficient, CognitiveCoefficient, MaximumInertialWeight, MinimumInertialWeight,
                                        VelocityFactor, VelocityConfinement);

    MTH::IGWO::AIGWOPlanner IGWOPlanner(PlannerConfiguration, PathConfiguration,
                                        MaximumWeight, MinimumWeight, MaximumInertialWeight, MinimumInertialWeight,
                                        VelocityFactor);

    MTH::ABC::AABCPlanner ABCPlanner(PlannerConfiguration, PathConfiguration);

    MTH::ABasePlanner *Planner = &IPSOPlanner; // Set the planner to IPSO Planner by default

    // Loop for each run
    for (int Run = 1; Run <= NRun; ++Run)
    {
        std::cout << "-------- " << "Run " << Run << " --------" << std::endl;

        std::vector<MTH::APoint> Waypoint; // Vector to store generated waypoints

        if (Planner->CreatePlan(CostMap, Start, Goal, Waypoint)) // Create plan using selected planner
        {
            // Output results
            std::cout << "Convergences: " << Planner->GetConvergence() << std::endl;
            std::cout << "Breakpoints: " << Planner->GetGlobalBestPosition() << std::endl;
            std::cout << "Waypoints: " << Waypoint << std::endl;
            std::cout << "Length (unit): " << Planner->GetPathLength() << std::endl;
        }

        Planner->Clear(); // Clear planner data for the next run
    }

    delete CostMap;

    return 1;
}
