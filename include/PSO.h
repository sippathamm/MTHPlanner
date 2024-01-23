//
// Created by Sippawit Thammawiset on 13/1/2024 AD.
//

#ifndef PSO_H
#define PSO_H

#include "Point.h"

#include <utility>
#include <vector>
#include <mutex>

#define LETHAL_COST         253

namespace PSO_Planner
{
    /**
     * @brief Individual particle contains N waypoints stored in position vector
    */
    typedef struct AParticle
    {
        AParticle () : BestFitnessValue((double)-INFINITY) {}

        std::vector<APoint> Position;
        std::vector<APoint> Velocity;

        std::vector<APoint> BestPosition;
        double BestFitnessValue;
    } AParticle;

    /**
     * @brief Particle Swarm Optimization for Global Path Planning
    */
    class APSO
    {
    public:
         /**
          * @brief Constructor
          *
          * @param LowerBound[in] An APoint object representing the lower bound of the search space.
          * @param UpperBound[in] An APoint object representing the upper bound of the search space.
          * @param MaxIteration[in] An integer specifying the maximum number of iterations.
          * @param Population[in] An integer specifying the population size.
          * @param Breakpoint[in] An integer specifying the number of breakpoints.
          * @param Waypoint[in] An integer specifying the number of waypoints.
          * @param InertialCoefficient[in] A double value representing the inertial coefficient.
          * @param SocialCoefficient[in] A double value representing the social coefficient.
          * @param CognitiveCoefficient[in] A double value representing the cognitive coefficient.
          * @param VelocityFactor[in] A double value representing the velocity factor for calculating allowable velocity.
          * @param ObstacleCostFactor[in] A double value representing the obstacle cost factor for thresholding obstacle cost in a costmap.
          * @param FitnessValueScalingFactor[in] A double value representing the scaling factor for calculating fitness value.
          * @param PenaltyScalingFactor[in] A double value representing the scaling factor for calculating penalty.
         */
         APSO (APoint LowerBound,
               APoint UpperBound,
               int MaxIteration, int Population, int Breakpoint, int Waypoint,
               double InertialCoefficient, double SocialCoefficient, double CognitiveCoefficient,
               double VelocityFactor = 0.5f, double ObstacleCostFactor = 1.00f,
               double FitnessValueScalingFactor = 1000.0f, double PenaltyScalingFactor = 100.0f);

        /**
         * @brief Destructor
        */
        ~APSO ();

        /**
         * @brief Initializes the position of a particle.
         *
         * @return An APoint object representing the initialized position of the particle.
         */
        APoint InitializePosition () const;

        /**
         * @brief Initializes the velocity of a particle.
         *
         * @return An APoint object representing the initialized velocity of the particle.
         */
        static APoint InitializeVelocity () ;

        /**
         * @brief Calculates the length of a path defined by a vector of waypoints.
         *
         * @param Length[out] Reference to a variable of double value that will store the calculated length of the path.
         * @param Waypoint[in] A constant reference to a vector of APoint objects representing the waypoints of the path.
         * @param WaypointIndex[in] An integer specifying the index of the waypoint.
         *
         * @return void
         */
        static void CalculateLength (double &Length, const std::vector<APoint> &Waypoint, int WaypointIndex);

        /**
         * @brief Converts 2D coordinates (X, Y) to a linear index.
         *
         * This method converts 2D coordinates (X, Y) to a linear index using a mapping scheme.
         * The linear index is calculated based on the specified X and Y coordinates.
         *
         * @param X[in] An integer representing the X-coordinate.
         * @param Y[in] An integer representing the Y-coordinate.
         *
         * @return An integer representing the linear index corresponding to the given 2D coordinates.
         */
        int XYToIndex (int X, int Y) const;

        /**
         * @brief Calculates a penalty value based on the provided costmap and waypoints.
         *
         * This method calculates a penalty value by evaluating the obstacle cost associated with the specified costmap
         * and the waypoints provided. The penalty value reflects the undesirability of the current path configuration.
         *
         * @param Costmap[in] A pointer points to an array of unsigned char values representing the costmap.
         * @param Waypoint[in] A constant reference to a vector of APoint objects representing the waypoints of the path.
         *
         * @return A double value representing the penalty associated with the current path configuration.
         */
        double Penalty (const unsigned char *Costmap, const std::vector<APoint> &Waypoint) const;

        /**
         * @brief Calculates the fitness value of a particle.
         *
         * @param Population[in] A pointer points to the AParticle object representing the particle of population.
         * @param Start[in] A constant reference to the starting point.
         * @param Goal[in] A constant reference to the goal point.
         * @param Waypoint[in] A constant reference to a vector of APoint objects representing the waypoints for the path.
          * @param Costmap[in] A pointer points to an array of unsigned char values representing the costmap.
         *
         * @return A double value representing the fitness value of the particle.
         */
        double FitnessFunction (AParticle *Population,
                                const APoint &Start, const APoint &Goal,
                                std::vector<APoint> &Waypoint,
                                const unsigned char *Costmap) const;

        /**
         * @brief Updates the velocity of a particle.
         *
         * @param Population[in] A pointer points to the AParticle objects representing the particle of population.
         * @param BreakpointIndex[in] An integer specifying the breakpoint index for the velocity update.
         *
         * @return An APoint object representing the updated velocity of the particle.
         */
        APoint UpdateVelocity (AParticle *Population, int BreakpointIndex) const;

        /**
         * @brief Updates the position of a particle.
         *
         * @param Population[in] A pointer points to the AParticle objects representing the particle of population.
         * @param BreakpointIndex[in] An integer specifying the breakpoint index for the position update.
         *
         * @return An APoint object representing the updated position of the particle.
         */
        APoint UpdatePosition (AParticle *Population, int BreakpointIndex) const;

        /**
        * @brief Generates a path from a given start point to a goal point within a costmap.
        *
        * This method generates a path within a costmap from a specified start point to a goal point using
        * the provided costmap data. The resulting path is stored in the provided vector.
        *
         * @param Costmap[in] A pointer points to an array of unsigned char values representing the costmap.
        * @param Start[in] A constant reference to the starting point.
         * @param Goal[in] A constant reference to the goal point.
        * @param Path[out] Reference to a vector of APoint objects that will store the generated path.
        *
        * @return Returns true if a valid path is found; otherwise, returns false.
        */
        bool CreatePlan (const unsigned char *Costmap,
                         const APoint &Start,
                         const APoint &Goal,
                         std::vector<APoint> &Path);

        void Optimize (AParticle *CurrentPopulation,
                       const APoint &Start, const APoint &Goal,
                       const unsigned char *Costmap);

    private:
        APoint LowerBound_, UpperBound_;
        int MaxIteration_, NPopulation_;
        int NBreakpoint_, NWaypoint_;
        double InertialCoefficient_, SocialCoefficient_, CognitiveCoefficient_;
        double VelocityFactor_;
        double ObstacleCostFactor_;
        double FitnessValueScalingFactor_;
        double PenaltyScalingFactor_;

        std::vector<AParticle> Population_;
        APoint Range_;
        int N_{};
        APoint MaximumVelocity_, MinimumVelocity_;
        std::vector<APoint> GlobalBestPosition_;
        double GlobalBestFitnessValue_ = (double)-INFINITY;
        std::mutex GlobalBestLock;

        std::vector<APoint> Waypoint_;
    };

    double GenerateRandom (double LowerBound = 0.0f, double UpperBound = 1.0f);
    std::vector<double> LinearInterpolation (double Begin, double End, int N);
}

#endif  // PSO_H