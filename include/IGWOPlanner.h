//
// Created by Sippawit Thammawiset on 6/2/2024 AD.
//

#ifndef IGWO_PLANNER_H
#define IGWO_PLANNER_H

#include "BasePlanner.h"

namespace MTH
{
    namespace IGWO
    {
        /**
         * @brief A struct representing a wolf in the IGWO algorithm.
         */
        typedef struct AWolf
        {
            AWolf () : Cost(2e10) {}

            std::vector<APoint> Position;
            double Cost;
        } AWolf;

        /**
         * @brief A struct representing leader wolves in the IGWO algorithm.
         */
        typedef struct ALeaderWolf
        {
            AWolf Alpha;    // 1st Best
            AWolf Beta;     // 2nd Best
            AWolf Delta;    // 3rd Best
        } ALeaderWolf;

        /**
         * @brief A class representing the Improved Grey Wolf Optimizer (IGWO) algorithm.
         */
        class AIGWOPlanner : public ABasePlanner
        {
        public:
            /**
             * @brief Constructor.
             *
             * @param PlannerConfiguration Configuration of optimization algorithm.
             * @param PathConfiguration Configuration of path.
             * @param MaximumWeight The maximum weight.
             * @param MinimumWeight The minimum weight.
             * @param MaximumInertialWeight Maximum inertial weight for velocity update.
             * @param MinimumInertialWeight Minimum inertial weight for velocity update.
             * @param VelocityFactor Factor for limiting velocity update.
             */
            AIGWOPlanner (const CONFIGURATION::APlannerConfiguration &PlannerConfiguration,
                          const CONFIGURATION::APathConfiguration &PathConfiguration,
                          double MaximumWeight = 2.2, double MinimumWeight = 0.02,
                          double MaximumInertialWeight = 0.9, double MinimumInertialWeight = 0.4,
                          double VelocityFactor = 0.5) :
                          ABasePlanner(PlannerConfiguration.LowerBound, PlannerConfiguration.UpperBound,
                                       PlannerConfiguration.MaximumIteration, PlannerConfiguration.NPopulation, PathConfiguration.NBreakpoint, PathConfiguration.NWaypoint,
                                       PathConfiguration.TrajectoryType),
                          MaximumWeight_(MaximumWeight),
                          MinimumWeight_(MinimumWeight),
                          MaximumInertialWeight_(MaximumInertialWeight),
                          MinimumInertialWeight_(MinimumInertialWeight),
                          VelocityFactor_(VelocityFactor),
                          InitialPositionType_(PlannerConfiguration.InitialPositionType),
                          Log_(PlannerConfiguration.Log)
            {
                std::cout << "[INFO] IGWO Planner instance has been created." << std::endl;
            }

            /**
             * @brief Destructor.
             */
            ~AIGWOPlanner () override = default;

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
            bool CreatePlan (const unsigned char *CostMap,
                             const APoint &Start,
                             const APoint &Goal,
                             std::vector<APoint> &Waypoint) override
            {
                // Set the inputs for the optimization process, including the cost map, start point, and goal point.
                this->CostMap_ = CostMap;
                this->Start_ = &Start;
                this->Goal_ = &Goal;

                // Initialize population
                this->Population_ = std::vector<AWolf> (NPopulation_);

                this->Range_ = this->UpperBound_ - this->LowerBound_;
                this->N_ = static_cast<int>(this->Range_.X * this->Range_.Y);

                this->MaximumVelocity_ = (this->UpperBound_ - this->LowerBound_) * this->VelocityFactor_;
                this->MinimumVelocity_ = MaximumVelocity_ * -1.0f;

                // Initialize wolves with random positions
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; ++PopulationIndex)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    std::vector<APoint> Position(this->NBreakpoint_);

                    // Generate random positions within bounds for each breakpoint
                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; ++BreakpointIndex)
                    {
                        APoint RandomPosition;

                        switch (this->InitialPositionType_)
                        {
                            case INITIAL_POSITION::DISTRIBUTED:
                                RandomPosition = GenerateDistributedPosition();
                                break;

                            case INITIAL_POSITION::CIRCULAR:
                                RandomPosition = GenerateCircularPosition();
                                break;

                            case INITIAL_POSITION::LINEAR:
                                RandomPosition = GenerateLinearPosition(BreakpointIndex);
                                break;

                            default:
                                RandomPosition = GenerateDistributedPosition();
                        }

                        Position[BreakpointIndex] = RandomPosition;
                    }

                    CurrentPopulation->Position = Position;

                    double Cost = ObjectiveFunction(CurrentPopulation->Position);
                    CurrentPopulation->Cost = Cost;

                    this->AverageCost_ += Cost;
                }

                // Compute average cost of the initial population.
                this->AverageCost_ /= this->NPopulation_;

                std::cout << "[INFO] IGWO Planner starts optimizing." << std::endl;

                // Start timing
                auto StartTime = std::chrono::high_resolution_clock::now();

                // Run optimization process for a maximum number of iterations
                for (int Iteration = 1; Iteration <= this->MaximumIteration_; ++Iteration)
                {
                    // Update the global best position
                    UpdateGlobalBestPosition();

                    // Calculate weight
                    CalculateWeight(Iteration);

                    // Optimize wolves positions
                    Optimize();

                    this->GlobalBestPosition_ = this->GlobalBest_.Alpha.Position;
                    this->GlobalBestCost_ = this->GlobalBest_.Alpha.Cost;

                    this->Convergence_.push_back(this->GlobalBestCost_);

                    if (this->Log_)
                    {
                        std::cout << "[INFO] Iteration: " << Iteration << " >>> \n\t"
                                  << "Best Cost: " << this->GlobalBestCost_ << " \n\t"
                                  << "Best Position: " << this->GlobalBestPosition_ <<
                                  std::endl;
                    }

                    // Update average cost for the next iteration.
                    this->NextAverageCost_ /= this->NPopulation_;
                    this->AverageCost_ = this->NextAverageCost_;
                    this->NextAverageCost_ = 0.0f;
                }

                // Stop timing
                auto StopTime = std::chrono::high_resolution_clock::now();

                // Calculate duration
                auto Duration = std::chrono::duration_cast<std::chrono::milliseconds>(StopTime - StartTime);

                std::cout << "[INFO] Completed, " << "taken " << Duration.count() << " milliseconds." << std::endl;

                double Length = 0.0f;

                switch (this->TrajectoryType_)
                {
                    case TRAJECTORY::LINEAR:
                        LinearPath(Length, Waypoint, this->GlobalBestPosition_);
                        break;

                    case TRAJECTORY::CUBIC_SPLINE:
                        CubicSplinePath(Length, Waypoint, this->GlobalBestPosition_);
                        break;

                    default:
                        CubicSplinePath(Length, Waypoint, this->GlobalBestPosition_);
                }

                // Check if the path is empty.
                if (Waypoint.empty())
                {
                    std::cerr << "[ERROR] Path not found!" << std::endl;

                    return STATE::FAILED;
                }

                this->PathLength_ = Length;

                return STATE::SUCCESS;
            }

            /**
             * @brief Clear the planner's state.
             */
            void Clear () override
            {
                // Clear the global best position.
                this->GlobalBest_.Alpha.Position.clear();
                this->GlobalBest_.Beta.Position.clear();
                this->GlobalBest_.Delta.Position.clear();

                // Set the global best cost to a large value.
                this->GlobalBest_.Alpha.Cost = 2e10;
                this->GlobalBest_.Beta.Cost = 2e10;
                this->GlobalBest_.Delta.Cost = 2e10;
                this->GlobalBestCost_ = 2e10;

                // Clear the convergence history.
                this->Convergence_.clear();
            }

        protected:
            double AlphaWeight_{}; /**< Weight for alpha wolf */
            double BetaWeight_{}; /**< Weight for beta wolf */
            double DeltaWeight_{}; /**< Weight for delta wolf */
            double AlphaGrowthFactor_ = 2.0; /**< Growth factor for alpha wolf weight. */
            double DeltaGrowthFactor_ = 3.0; /**< Growth factor for delta wolf weight. */
            double MaximumWeight_; /**< Maximum weight */
            double MinimumWeight_; /**< Minimum weight */
            double Theta_ = 2.2; /**< Scaling factor used to regulate the form of the function curve for the inertial weight calculation. */
            double K_ = 1.5; /**< Scaling factor used to regulate the form of the function curve for the inertial weight calculation. */
            double InertialWeight_{}; /**< Current inertial weight. */
            double MaximumInertialWeight_; /**< Maximum inertial weight for velocity update. */
            double MinimumInertialWeight_; /**< Minimum inertial weight for velocity update. */
            double VelocityFactor_; /**< Factor for limiting velocity update. */

            std::vector<AWolf> Population_; /**< Vector containing wolves. */
            APoint MaximumVelocity_; /**< Vector containing maximum velocity for each breakpoint. */
            APoint MinimumVelocity_; /**< Vector containing minimum velocity for each breakpoint. */

            ALeaderWolf GlobalBest_; /**< Global best wolves found by the algorithm. */
            double AverageCost_ = 0.0; /**< Average cost of the population. */
            double NextAverageCost_ = 0.0; /**< Next average cost of the population. */

            INITIAL_POSITION_TYPE InitialPositionType_; /**< Type of initial position distribution. */
            bool Log_; /**< Flag indicating whether to log information during optimization. */

        private:
            /**
             * @brief Update the global best position.
             *
             * This function iterates through the population and updates the global best position
             * based on the cost of each individual in the population.
             */
            void UpdateGlobalBestPosition ()
            {
                // Iterate over each wolf in the population
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; ++PopulationIndex)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    // Update alpha wolf's position and cost if the current wolf has a lower cost than alpha's cost
                    if (CurrentPopulation->Cost < GlobalBest_.Alpha.Cost)
                    {
                        GlobalBest_.Alpha.Position = CurrentPopulation->Position;
                        GlobalBest_.Alpha.Cost = CurrentPopulation->Cost;
                    }

                    // Update beta wolf's position and cost if the current wolf has a lower cost than beta's cost
                    if (CurrentPopulation->Cost > GlobalBest_.Alpha.Cost &&
                        CurrentPopulation->Cost < GlobalBest_.Beta.Cost)
                    {
                        GlobalBest_.Beta.Position = CurrentPopulation->Position;
                        GlobalBest_.Beta.Cost = CurrentPopulation->Cost;
                    }

                    // Update delta wolf's position and cost if the current wolf has a lower cost than delta's cost
                    if (CurrentPopulation->Cost > GlobalBest_.Alpha.Cost &&
                        CurrentPopulation->Cost > GlobalBest_.Beta.Cost &&
                        CurrentPopulation->Cost < GlobalBest_.Delta.Cost)
                    {
                        GlobalBest_.Delta.Position = CurrentPopulation->Position;
                        GlobalBest_.Delta.Cost = CurrentPopulation->Cost;
                    }
                }
            }

            /**
             * @brief Calculate the weights based on the current iteration.
             *
             * This function calculates the weights (alpha, beta, and delta)
             * based on the current iteration.
             *
             * @param Iteration The current iteration.
             *
             * @note The weight calculation is extracted from the paper "Improved GWO algorithm
             * for optimal design of truss structures" by A. Kaveh and P. Zakian.
             * The weight equations are derived from Equation (8), (9), and (10) of their paper.
             * Link to the paper: https://link.springer.com/article/10.1007/s00366-017-0567-1
             */
            void CalculateWeight (int Iteration)
            {
                // Calculate alpha weight
                this->AlphaWeight_ = this->MaximumWeight_ *
                                     exp(pow(static_cast<double>(Iteration) / this->MaximumIteration_, this->AlphaGrowthFactor_) *
                                         log(this->MinimumWeight_ / this->MaximumWeight_));

                // Calculate delta weight
                this->DeltaWeight_ = this->MaximumWeight_ *
                                     exp(pow(static_cast<double>(Iteration) / this->MaximumIteration_, this->DeltaGrowthFactor_) *
                                         log(this->MinimumWeight_ / this->MaximumWeight_));

                // Calculate beta weight as the average of alpha and delta weights
                this->BetaWeight_ = (this->AlphaWeight_ + this->DeltaWeight_) * 0.5;
            }

            /**
             * @brief Calculates the inertial weight for the current population using the multi-modal adaptive function (MAF) with tangent function.
             *
             * From the paper "An Improved grey wolf optimizer with weighting functions and its application to Unmanned Aerial Vehicles path planning"
             * by Hongran Li, Tieli Lv, Yuchao Shui, Jian Zhang, Heng Zhang, Hui Zhao, and Saibao Ma.
             * Link to the paper: https://www.sciencedirect.com/science/article/pii/S0045790623003178
             *
             * @param CurrentPopulation Pointer to the current population.
             */
            void CalculateInertialWeight (AWolf *CurrentPopulation)
            {
                // Equation (13) from the paper "An Improved grey wolf optimizer with weighting functions and its application to Unmanned Aerial Vehicles path planning"
                double H = this->K_ * ((0.0 - CurrentPopulation->Cost) / this->AverageCost_);

                // Equation (12) from the paper "An Improved grey wolf optimizer with weighting functions and its application to Unmanned Aerial Vehicles path planning"
                this->InertialWeight_ = ((this->MaximumInertialWeight_ + this->MinimumInertialWeight_) * 0.5f) +
                                        (this->MaximumInertialWeight_ - this->MinimumInertialWeight_) *
                                        (H / (std::hypot(1.0f, H))) * tan(this->Theta_);
            }

            /**
             * @brief Optimize the positions of wolves in the population.
             *
             * This function optimizes the positions of wolves in the population.
             * It calculates the GWO and DLH positions of each wolf, evaluates the cost of the calculated positions,
             * updates the best position of each wolf based on the comparison of GWO and DLH costs.
             *
             * @note - The IGWO algorithm incorporates an additional movement strategy named
             * dimension learning-based hunting (DLH) search strategy.
             * In DLH, each individual wolf is learned by its neighbors to be another candidate
             * for the new position of Xi (t). The information about IGWO algorithm and DLH is based on the paper "An improved grey wolf optimizer for solving engineering problems"
             * by Mohammad H. Nadimi-Shahraki, Shokooh Taghian, and Seyedali Mirjalili.
             * Link to the paper: https://www.sciencedirect.com/science/article/pii/S0957417420307107
             * - The IGWO-WFs algorithm initially refers to the PSO algorithm and introduces a weight update function to transform the position and update rule.
             * After the position and rule updates, the DLH search strategy is adopted. During each iteration, the weights are continuously updated based on the progress of the algorithm and search results,
             * with the goal of improving the convergence velocity and search effectiveness of the algorithm.
             * The information is extracted from the paper "An Improved grey wolf optimizer with weighting functions and its application to Unmanned Aerial Vehicles path planning"
             * by Hongran Li, Tieli Lv, Yuchao Shui, Jian Zhang, Heng Zhang, Hui Zhao, and Saibao Ma.
             * Link to the paper: https://www.sciencedirect.com/science/article/pii/S0045790623003178
             */
            void Optimize ()
            {
                // Iterate over each wolf in the population
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; ++PopulationIndex)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    CalculateInertialWeight(CurrentPopulation);

                    double Radius = 0.0;
                    std::vector<APoint> GWOPosition(this->NBreakpoint_);

                    // Iterate over each breakpoint of the wolf
                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; ++BreakpointIndex)
                    {
                        // Generate random values for alpha, beta, and delta
                        double A1 = this->AlphaWeight_ * (2.0 * GenerateRandom(0.0, 1.0) - 1.0);
                        double A2 = this->BetaWeight_ * (2.0 * GenerateRandom(0.0, 1.0) - 1.0);
                        double A3 = this->DeltaWeight_ * (2.0 * GenerateRandom(0.0, 1.0) - 1.0);

                        // Generate random values for C1, C2, and C3
                        double C1 = 2.0 * GenerateRandom(0.0, 1.0);
                        double C2 = 2.0 * GenerateRandom(0.0, 1.0);
                        double C3 = 2.0 * GenerateRandom(0.0, 1.0);

                        // Calculate new position components
                        APoint X1 = GlobalBest_.Alpha.Position[BreakpointIndex] -
                                    Absolute((GlobalBest_.Alpha.Position[BreakpointIndex] - CurrentPopulation->Position[BreakpointIndex]) * C1) * A1;
                        APoint X2 = GlobalBest_.Beta.Position[BreakpointIndex] -
                                    Absolute((GlobalBest_.Beta.Position[BreakpointIndex] - CurrentPopulation->Position[BreakpointIndex]) * C2) * A2;
                        APoint X3 = GlobalBest_.Delta.Position[BreakpointIndex] -
                                    Absolute((GlobalBest_.Delta.Position[BreakpointIndex] - CurrentPopulation->Position[BreakpointIndex]) * C3) * A3;

                        // Calculate new position
                        APoint X = (X1 + X2 + X3) / 3.0;

                        // Generate random values for R1, R2, and R3
                        double R1 = GenerateRandom(0.0, 1.0);
                        double R2 = GenerateRandom(0.0, 1.0);
                        double R3 = GenerateRandom(0.0, 1.0);

                        // Calculate new velocity
                        // Equation (9) from the paper "An Improved grey wolf optimizer with weighting functions and its application to Unmanned Aerial Vehicles path planning"
                        APoint NewVelocity = (((X1 - X) * C1 * R1) + ((X2 - X) * C2 * R2) + ((X3 - X) * C3 * R3)) * this->InertialWeight_;

                        // Clamp the new velocity to stay within specified bounds
                        NewVelocity.X = CLAMP(NewVelocity.X, this->MinimumVelocity_.X, this->MaximumVelocity_.X);
                        NewVelocity.Y = CLAMP(NewVelocity.Y, this->MinimumVelocity_.Y, this->MaximumVelocity_.Y);

                        // Equation (8) from the paper "An Improved grey wolf optimizer with weighting functions and its application to Unmanned Aerial Vehicles path planning"
                        APoint TemporaryNewPosition = X + NewVelocity;

                        // Check if the temporary new position is out of bounds
                        if (IS_OUT_OF_BOUND(TemporaryNewPosition.X, this->LowerBound_.X, this->UpperBound_.X) ||
                            IS_OUT_OF_BOUND(TemporaryNewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y))
                        {
                            // Apply velocity confinement
                            APoint VelocityConfinement = NewVelocity * -GenerateRandom(0.0, 1.0);

                            NewVelocity = VelocityConfinement;
                        }

                        APoint NewPosition = X + NewVelocity;

                        // Clamp the new position to stay within specified bounds
                        NewPosition.X = CLAMP(NewPosition.X, this->LowerBound_.X, this->UpperBound_.X);
                        NewPosition.Y = CLAMP(NewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y);

                        // Equation (10) from the paper "An improved grey wolf optimizer for solving engineering problems"
                        Radius += std::pow(CurrentPopulation->Position[BreakpointIndex].X - NewPosition.X, 2) +
                                  std::pow(CurrentPopulation->Position[BreakpointIndex].Y - NewPosition.Y, 2);

                        // Store GWO position
                        GWOPosition[BreakpointIndex] = NewPosition;
                    }

                    Radius = sqrt(Radius);

                    // Get the index of neighborhood and calculate the DLH position
                    std::vector<int> Index = GetNeighborHoodIndex(CurrentPopulation, Radius);
                    std::vector<APoint> DLHPosition = CalculateDLHPosition(CurrentPopulation, Index);

                    // Calculate costs for GWO and DLH positions
                    double GWOCost = ObjectiveFunction(GWOPosition);
                    double DLHCost = ObjectiveFunction(DLHPosition);

                    // Update the wolf's position and cost based on the comparison of GWO and DLH costs
                    // Equation (13) from the paper "An improved grey wolf optimizer for solving engineering problems"
                    if (GWOCost < DLHCost)
                    {
                        CurrentPopulation->Position = GWOPosition;
                        CurrentPopulation->Cost = GWOCost;

                        this->AverageCost_ += GWOCost;
                    }
                    else
                    {
                        CurrentPopulation->Position = DLHPosition;
                        CurrentPopulation->Cost = DLHCost;

                        this->AverageCost_ += DLHCost;
                    }
                }
            }

            /**
             * @brief Get the indices of wolves within the neighborhood of a given wolf.
             *
             * This function calculates the indices of wolves within the neighborhood of a given wolf
             * based on the provided radius.
             *
             * @param CurrentPopulation Pointer to the current population.
             * @param Radius The radius used to define the neighborhood.
             *
             * @return A vector containing the indices of wolves within the neighborhood.
             */
            std::vector<int> GetNeighborHoodIndex (AWolf *CurrentPopulation, double &Radius)
            {
                std::vector<int> Index;

                // Iterate over each wolf in the population
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; ++PopulationIndex)
                {
                    auto *AnotherPopulation = &this->Population_[PopulationIndex];

                    double Distance = 0.0;

                    // Calculate the distance between the current wolf and neighbor wolf in the population
                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; ++BreakpointIndex)
                    {
                        Distance += std::pow(CurrentPopulation->Position[BreakpointIndex].X - AnotherPopulation->Position[BreakpointIndex].X, 2) +
                                    std::pow(CurrentPopulation->Position[BreakpointIndex].Y - AnotherPopulation->Position[BreakpointIndex].Y, 2);

                    }

                    Distance = sqrt(Distance);

                    // Equation (11) from the paper "An improved grey wolf optimizer for solving engineering problems"
                    if (Distance <= Radius)
                    {
                        Index.push_back(PopulationIndex);
                    }
                }

                return Index;
            }

            /**
             * @brief Calculate the DLH position for a wolf.
             *
             * This function calculates the DLH position for a wolf based on its neighbors' positions.
             *
             * @param CurrentPopulation Pointer to the current population.
             * @param Index Vector containing the indices of wolves within the neighborhood.
             *
             * @return A vector representing the DLH position for the current wolf.
             */
            std::vector<APoint> CalculateDLHPosition (AWolf *CurrentPopulation, const std::vector<int> &Index)
            {
                std::vector<APoint> DLHPosition(this->NBreakpoint_);

                // Iterate over each wolf in the population
                for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; ++BreakpointIndex)
                {
                    // Select a random index from the neighborhood
                    int NeighborIndex = GenerateRandom((int)Index.size() - 1);

                    // Select a random index from the population
                    int PopulationIndex = GenerateRandom(this->NPopulation_ - 1);

                    // Equation (12) from the paper "An improved grey wolf optimizer for solving engineering problems"
                    APoint DLH = CurrentPopulation->Position[BreakpointIndex] +
                                 (this->Population_[Index[NeighborIndex]].Position[BreakpointIndex] -
                                 this->Population_[PopulationIndex].Position[BreakpointIndex]) * GenerateRandom(0.0f, 1.0f);

                    // Clamp the DLH position to stay within specified bounds
                    DLH.X = CLAMP(DLH.X, this->LowerBound_.X, this->UpperBound_.X);
                    DLH.Y = CLAMP(DLH.Y, this->LowerBound_.Y, this->UpperBound_.Y);

                    // Store DLH position
                    DLHPosition[BreakpointIndex] = DLH;
                }

                return DLHPosition;
            }
        };
    } // IGWO
} // MTH

#endif // IGWO_PLANNER_H
