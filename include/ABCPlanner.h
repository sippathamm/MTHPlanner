//
// Created by Sippawit Thammawiset on 11/2/2024 AD.
//

#ifndef ABC_PLANNER_H
#define ABC_PLANNER_H

#include "BasePlanner.h"

namespace MTH
{
    namespace ABC
    {
        /**
         * @brief A struct representing a bee in the ABC algorithm.
         */
        typedef struct ABee
        {
            ABee () : Cost(0.0), FitnessValue(0.0), Probability(0.0), Trial(0) {}

            std::vector<APoint> Position;
            double Cost;
            double FitnessValue;

            double Probability;
            int Trial;
        } ABee;

        /**
         * @brief A class representing the Artificial Bee Colony (ABC) algorithm.
         */
        class AABCPlanner : public ABasePlanner
        {
        public:
            /**
             * @brief Constructor.
             *
             * @param LowerBound Lower bound of the search space.
             * @param UpperBound Upper bound of the search space.
             * @param MaximumIteration Maximum number of iterations.
             * @param NPopulation Population size.
             * @param NBreakpoint Number of breakpoints.
             * @param NWaypoint Number of waypoints.
             * @param InitialPositionType Type of initial position distribution.
             * @param TrajectoryType Type of trajectory.
             * @param Log Flag indicating whether to log information during optimization.
             */
            AABCPlanner (const APoint &LowerBound, const APoint &UpperBound,
                         int MaximumIteration, int NPopulation, int NBreakpoint, int NWaypoint,
                         INITIAL_POSITION_TYPE InitialPositionType = INITIAL_POSITION::DISTRIBUTED,
                         TRAJECTORY_TYPE TrajectoryType = TRAJECTORY::CUBIC_SPLINE,
                         bool Log = true) :
                         ABasePlanner(LowerBound, UpperBound,
                                      MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                      TrajectoryType),
                         InitialPositionType_(InitialPositionType),
                         Log_(Log)
            {
                std::cout << "[INFO] ABC Planner instance has been created." << std::endl;
            }

            /**
             * @brief Destructor.
             */
            ~AABCPlanner () override = default;

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

                // Calculate the number of employed and onlooker bees, as well as the trial limit and initialize the food source.
                this->NEmployedBee_ = this->NPopulation_ * 0.5;
                this->NOnLookerBee_ = this->NEmployedBee_;
                this->TrialLimit_ = this->NPopulation_ * this->NBreakpoint_ * 0.5;
                this->FoodSource_ = std::vector<ABee>(this->NEmployedBee_);

                this->Range_ = this->UpperBound_ - this->LowerBound_;
                this->N_ = static_cast<int>(this->Range_.X * this->Range_.Y);

                // Initialize particles with random positions.
                for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; ++EmployedBeeIndex)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    std::vector<APoint> Position(this->NBreakpoint_);

                    // Generate random positions within bounds for each breakpoint.
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

                    CurrentEmployedBee->Position = Position;

                    double Cost = ObjectiveFunction(CurrentEmployedBee->Position);
                    double FitnessValue = FitnessFunction(Cost);

                    CurrentEmployedBee->Cost = Cost;
                    CurrentEmployedBee->FitnessValue = FitnessValue;

                    // Update global best position and cost.
                    if (CurrentEmployedBee->Cost < this->GlobalBestCost_)
                    {
                        this->GlobalBestPosition_ = CurrentEmployedBee->Position;
                        this->GlobalBestCost_ = CurrentEmployedBee->Cost;
                    }
                }

                std::cout << "[INFO] ABC Planner starts optimizing." << std::endl;

                // Start timing
                auto StartTime = std::chrono::high_resolution_clock::now();

                // Run optimization process for a maximum number of iterations.
                for (int Iteration = 1; Iteration <= this->MaximumIteration_; ++Iteration)
                {
                    // Optimize bee positions.
                    Optimize();

                    this->Convergence_.push_back(this->GlobalBestCost_);

                    if (this->Log_)
                    {
                        std::cout << "[INFO] Iteration: " << Iteration << " >>> \n\t"
                                  << "Best Cost: " << this->GlobalBestCost_ << " \n\t"
                                  << "Best Position: " << this->GlobalBestPosition_ <<
                                  std::endl;
                    }
                }

                // Stop timing
                auto StopTime = std::chrono::high_resolution_clock::now();

                // Calculate duration
                auto Duration = std::chrono::duration_cast<std::chrono::milliseconds>(StopTime - StartTime);

                std::cout << "[INFO] Completed, " << "taken " << Duration.count() << " milliseconds." << std::endl;

                double Length = 0.0;

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
                    std::cerr << "[INFO] Path not found!" << std::endl;

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
                // Clear the global best position and set the global best cost to a large value.
                this->GlobalBestPosition_.clear();
                this->GlobalBestCost_ = 2e10;

                // Clear the convergence history.
                this->Convergence_.clear();
            }

        private:
            int NEmployedBee_{}; /**< Number of employed bees. */
            int NOnLookerBee_{}; /**< Number of onlooker bees. */
            int NScoutBee = 1; /**< Number of scout bees. */
            int TrialLimit_{}; /**< Trial limit for employed and onlooker bees. */

            std::vector<ABee> FoodSource_; /**< Vector containing bees. */
            double MaximumFitnessValue_ = -2e10; /**< Maximum fitness value. */

            INITIAL_POSITION_TYPE InitialPositionType_; /**< Type of initial position distribution. */
            bool Log_; /**< Flag indicating whether to log information during optimization. */

            /**
             * @brief Compute the fitness value based on the cost.
             *
             * If the cost is greater than or equal to 0, the fitness value is computed as 1 / (1 + Cost).
             * If the cost is negative, the fitness value is computed as 1 + |Cost|.
             *
             * @param Cost The cost value.
             *
             * @return The computed fitness value.
             */
            static double FitnessFunction (double Cost)
            {
                if (Cost >= 0)
                {
                    return 1.0 / (1.0 + Cost);
                }

                return 1.0 + abs(Cost);
            }

            /**
             * @brief Optimize the positions of bees in the population.
             *
             * Sends employed bees, calculates probabilities, sends onlooker bees, and sends scout bees.
             * Updates the global best position and cost based on the comparison with each employed bee's cost.
             */
            void Optimize ()
            {
                // Send employed bees
                SendEmployedBee();

                // Calculate probabilities for employed bees
                CalculateProbability();

                // Send onlooker bees based on the probabilities
                SendOnLookerBee();

                // Send scout bees to explore new areas
                SendScoutBee();

                // Update global best position and cost.
                for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; ++EmployedBeeIndex)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    if (CurrentEmployedBee->Cost < this->GlobalBestCost_)
                    {
                        // Update global best position and cost if a better solution is found
                        this->GlobalBestPosition_ = CurrentEmployedBee->Position;
                        this->GlobalBestCost_ = CurrentEmployedBee->Cost;
                    }
                }

                this->MaximumFitnessValue_ = -2e10;
            }

            /**
             * @brief Send employed bees to explore new positions.
             *
             * Each employed bee randomly selects a breakpoint and a partner bee.
             * It updates its position based on the partner bee's position.
             * The updated position is then evaluated for cost and fitness.
             * If the fitness value improves, the employed bee accepts the new position; otherwise, it increases its trial count.
             * The maximum fitness value is updated based on the best fitness value encountered so far.
             */
            void SendEmployedBee ()
            {
                for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; ++EmployedBeeIndex)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    // Select a random breakpoint
                    int BreakpointIndex = GenerateRandom(this->NBreakpoint_ - 1);

                    // Select a partner bee (different from the current employed bee)
                    int PartnerBeeIndex;
                    do
                    {
                        PartnerBeeIndex = GenerateRandom(this->NEmployedBee_ - 1);
                    } while (PartnerBeeIndex == EmployedBeeIndex);

                    // Update the position based on the partner bee's position
                    std::vector<APoint> UpdatedPosition = CurrentEmployedBee->Position;
                    UpdatedPosition[BreakpointIndex] = CurrentEmployedBee->Position[BreakpointIndex] +
                                                       (CurrentEmployedBee->Position[BreakpointIndex] -
                                                       this->FoodSource_[PartnerBeeIndex].Position[BreakpointIndex]) *
                                                       ((GenerateRandom(0.0, 1.0) - 0.5) * 2);

                    // Clamp the updated position to stay within specified bounds
                    UpdatedPosition[BreakpointIndex].X = CLAMP(UpdatedPosition[BreakpointIndex].X,
                                                               this->LowerBound_.X,
                                                               this->UpperBound_.X);
                    UpdatedPosition[BreakpointIndex].Y = CLAMP(UpdatedPosition[BreakpointIndex].Y,
                                                               this->LowerBound_.Y,
                                                               this->UpperBound_.Y);

                    // Evaluate the updated position
                    std::vector<APoint> Waypoint;
                    double Cost = ObjectiveFunction(UpdatedPosition);
                    double FitnessValue = FitnessFunction(Cost);

                    // Update the employed bee if the fitness improves
                    if (FitnessValue > CurrentEmployedBee->FitnessValue)
                    {
                        CurrentEmployedBee->Position = UpdatedPosition;
                        CurrentEmployedBee->Cost = Cost;
                        CurrentEmployedBee->FitnessValue = FitnessValue;
                        CurrentEmployedBee->Trial = 0;
                    }
                    else
                    {
                        // Increase the trial count if the fitness does not improve
                        CurrentEmployedBee->Trial++;
                    }

                    // Update the maximum fitness value encountered so far
                    this->MaximumFitnessValue_ = std::max<double>(this->MaximumFitnessValue_,
                                                                  CurrentEmployedBee->FitnessValue);
                }
            }

            /**
             * @brief Calculate the probability of employed bees for onlooker bees selection.
             *
             * For each employed bee, calculate its probability based on its fitness value relative to the maximum fitness value.
             */
            void CalculateProbability ()
            {
                for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; ++EmployedBeeIndex)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    // Calculate probability based on fitness value relative to the maximum fitness value
                    CurrentEmployedBee->Probability = (0.9f * (CurrentEmployedBee->FitnessValue / this->MaximumFitnessValue_)) + 0.1f;
                }
            }

            /**
             * @brief Send onlooker bees to explore promising food sources based on employed bees' probabilities.
             *
             * Continuously select onlooker bees until the required number of onlooker bees is reached.
             * Each onlooker bee selects a food source based on the employed bees' probabilities.
             * If selected, an onlooker bee explores the selected food source.
             */
            void SendOnLookerBee ()
            {
                int EmployedBeeIndex = 0;
                int OnLookerBeeIndex = 0;

                // Iterate until the required number of onlooker bees is reached
                while (OnLookerBeeIndex < this->NOnLookerBee_)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    // Generate a random probability to determine if the current employed bee will be selected
                    double RandomProbability = GenerateRandom(0.0, 1.0);

                    // If the random probability is less than the current employed bee's probability, select the employed bee
                    if (RandomProbability < CurrentEmployedBee->Probability)
                    {
                        // Select a random breakpoint
                        int BreakpointIndex = GenerateRandom(this->NBreakpoint_ - 1);

                        // Select a partner bee (different from the current employed bee)
                        int PartnerBeeIndex;

                        do
                        {
                            PartnerBeeIndex = GenerateRandom(this->NEmployedBee_ - 1);
                        } while (PartnerBeeIndex == EmployedBeeIndex);

                        // Update the position based on the partner bee's position
                        std::vector<APoint> UpdatedPosition = CurrentEmployedBee->Position;
                        UpdatedPosition[BreakpointIndex] = CurrentEmployedBee->Position[BreakpointIndex] +
                                                           (CurrentEmployedBee->Position[BreakpointIndex] -
                                                            this->FoodSource_[PartnerBeeIndex].Position[BreakpointIndex]) *
                                                           ((GenerateRandom(0.0f, 1.0f) - 0.5f) * 2);

                        // Clamp the updated position to stay within specified bounds
                        UpdatedPosition[BreakpointIndex].X = CLAMP(UpdatedPosition[BreakpointIndex].X,
                                                                   this->LowerBound_.X,
                                                                   this->UpperBound_.X);
                        UpdatedPosition[BreakpointIndex].Y = CLAMP(UpdatedPosition[BreakpointIndex].Y,
                                                                   this->LowerBound_.Y,
                                                                   this->UpperBound_.Y);

                        // Evaluate the updated position
                        std::vector<APoint> Waypoint;
                        double Cost = ObjectiveFunction(UpdatedPosition);
                        double FitnessValue = FitnessFunction(Cost);

                        // Update the employed bee if the fitness improves
                        if (FitnessValue > CurrentEmployedBee->FitnessValue)
                        {
                            CurrentEmployedBee->Position = UpdatedPosition;
                            CurrentEmployedBee->Cost = Cost;
                            CurrentEmployedBee->FitnessValue = FitnessValue;
                            CurrentEmployedBee->Trial = 0;
                        }
                        else
                        {
                            // Increase the trial count if the fitness does not improve
                            CurrentEmployedBee->Trial++;
                        }

                        // Increment the index of onlooker bees
                        OnLookerBeeIndex++;
                    }

                    // Move to the next employed bee, cyclically
                    EmployedBeeIndex = (EmployedBeeIndex + 1) % this->NOnLookerBee_;
                }
            }

            /**
             * @brief Send scout bees to explore new food sources if employed bees exceed the trial limit.
             *
             * For each scout bee, if the trial count of an employed bee exceeds the trial limit,
             * replace its position with a randomly generated position within the search space.
             * Reset the cost, fitness value, and trial count of the employed bee.
             */
            void SendScoutBee ()
            {
                // Iterate over each scout bee
                for (int ScoutBeeIndex = 0; ScoutBeeIndex < this->NScoutBee; ++ScoutBeeIndex)
                {
                    // Iterate over each employed bee
                    for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; ++EmployedBeeIndex)
                    {
                        auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                        // If the trial count of the employed bee exceeds the trial limit, generate a new random position
                        if (CurrentEmployedBee->Trial >= this->TrialLimit_)
                        {
                            std::vector<APoint> Position(this->NBreakpoint_);

                            // Generate random positions within bounds for each breakpoint.
                            for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; ++BreakpointIndex)
                            {
                                APoint RandomPosition;
                                RandomPosition.X = GenerateRandom(this->LowerBound_.X, this->UpperBound_.X);
                                RandomPosition.Y = GenerateRandom(this->LowerBound_.Y, this->UpperBound_.Y);

                                Position[BreakpointIndex] = RandomPosition;
                            }

                            // Update the employed bee's position, cost, fitness value, and trial count
                            CurrentEmployedBee->Position = Position;

                            double Cost = ObjectiveFunction(CurrentEmployedBee->Position);
                            double FitnessValue = FitnessFunction(Cost);
                            CurrentEmployedBee->Cost = Cost;
                            CurrentEmployedBee->FitnessValue = FitnessValue;
                            CurrentEmployedBee->Trial = 0;
                        }
                    }
                }
            }
        };
    } // ABC
} // MTH

#endif // ABC_PLANNER_H
