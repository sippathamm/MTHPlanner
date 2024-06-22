//
// Created by Sippawit Thammawiset on 22/6/2024 AD.
//

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "Point.h"
#include "Utility.h"

namespace MTH
{
    namespace CONFIGURATION
    {
        typedef struct APlannerConfiguration
        {
            APoint LowerBound; // Lower bound of the search space
            APoint UpperBound; // Upper bound of the search space
            int MaximumIteration; // Maximum number of iterations
            int NPopulation; // Population size
            INITIAL_POSITION_TYPE InitialPositionType; // Type of initial position distribution
            bool Log; // Flag indicating whether to log information during optimization
        } APlannerConfiguration;

        typedef struct APathConfiguration
        {
            int NBreakpoint; // Number of breakpoints
            int NWaypoint; // Number of waypoints
            TRAJECTORY_TYPE TrajectoryType; // Type of trajectory
        } APathConfiguration;
    }
}

#endif // CONFIGURATION_H
