//
// Created by Sippawit Thammawiset on 3/2/2024 AD.
//

#include "Print.h"
#include "CostMapLoader.h"
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

    MTH::APoint LowerBound(0.0, 0.0); // Lower bound of the search space
    MTH::APoint UpperBound(Width, Height); // Upper bound of the search space
    int MaximumIteration = 100; // Maximum number of iterations for optimization
    int NPopulation = 100; // Population size
    int NBreakpoint = 3; // Number of breakpoints
    int NInterpolationPoint = 5; // Number of interpolation points
    int NWaypoint = 1 + (NBreakpoint + 1) * NInterpolationPoint; // Number of waypoints
    double MaximumInertialWeight = 0.9; // Maximum inertial weight
    double MinimumInertialWeight = 0.4; // Minimum inertial weight
    double VelocityFactor = 0.5; // Velocity factor for limiting velocity update
    MTH::INITIAL_POSITION_TYPE InitialPositionType = MTH::INITIAL_POSITION::CIRCULAR; // Type of initial position distribution
    MTH::TRAJECTORY_TYPE TrajectoryType = MTH::TRAJECTORY::CUBIC_SPLINE; // Type of trajectory
    bool Log = false; // Flag indicating whether to log information during optimization

    // PSO parameters
    double SocialCoefficient = 2.0f; // Social coefficient
    double CognitiveCoefficient = 1.3f; // Cognitive coefficient
    int VelocityConfinement = MTH::IPSO::VELOCITY_CONFINEMENT::HYPERBOLIC; // Velocity confinement type

    // GWO parameters
    double MaximumWeight = 2.2; // Maximum weight
    double MinimumWeight = 0.02; // Minimum weight

    int NRun = 1; // Number of runs

    // Initialize planners
    MTH::IPSO::AIPSOPlanner IPSOPlanner(LowerBound, UpperBound, MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                        SocialCoefficient, CognitiveCoefficient, MaximumInertialWeight, MinimumInertialWeight,
                                        VelocityFactor, VelocityConfinement, InitialPositionType, TrajectoryType, Log);

    MTH::IGWO::AIGWOPlanner IGWOPlanner(LowerBound, UpperBound, MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                        MaximumWeight, MinimumWeight, MaximumInertialWeight, MinimumInertialWeight,
                                        VelocityFactor, InitialPositionType, TrajectoryType, Log);

    MTH::ABC::AABCPlanner ABCPlanner(LowerBound, UpperBound, MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                     InitialPositionType, TrajectoryType, Log);

    MTH::ABasePlanner *Planner = &IPSOPlanner; // Set the planner to IPSO Planner by default

    // Loop for each run
    for (int Run = 1; Run <= NRun; ++Run)
    {
        std::cout << "-------- " << "Run " << Run << " --------" << std::endl;

        std::vector<MTH::APoint> Waypoint; // Vector to store generated waypoints

        if (Planner->CreatePlan(CostMap, Start, Goal, Waypoint)) // Create plan using selected planner
        {
            // Output results
            std::cout << "Convergence: " << Planner->GetConvergence() << std::endl;
            std::cout << "Breakpoint: " << Planner->GetGlobalBestPosition() << std::endl;
            std::cout << "Waypoint: " << Waypoint << std::endl;
            std::cout << "Length (px): " << Planner->GetPathLength() << std::endl;
        }

        Planner->Clear(); // Clear planner data for the next run
    }

    delete CostMap;

    return 1;
}
