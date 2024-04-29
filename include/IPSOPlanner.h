//
// Created by Sippawit Thammawiset on 17/2/2024 AD.
//

#ifndef IPSO_PLANNER_H
#define IPSO_PLANNER_H

#include "BasePlanner.h"

namespace MTH
{
    namespace IPSO
    {
        namespace VELOCITY_CONFINEMENT
        {
            /**
             * @brief An enum defining different types of velocity confinement methods.
             */
            enum VELOCITY_CONFINEMENT
            {
                RANDOM_BACK = 0,
                HYPERBOLIC = 1,
                MIXED = 2
            };
        }

        /**
         * @brief A struct representing a particle in the IPSO algorithm.
         */
        typedef struct AParticle
        {
            AParticle () : BestCost(2e10), Cost(0.0) {}

            std::vector<APoint> Position;
            std::vector<APoint> Velocity;
            double Cost;

            std::vector<APoint> BestPosition;
            double BestCost;

            std::vector<APoint> Feedback;
        } AParticle;

        /**
         * @brief A class representing the Improved Particle Swarm Optimization (IPSO) algorithm.
         */
        class AIPSOPlanner : public ABasePlanner
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
             * @param SocialCoefficient Social coefficient for velocity update.
             * @param CognitiveCoefficient Cognitive coefficient for velocity update.
             * @param MaximumInertialWeight Maximum inertial weight for velocity update.
             * @param MinimumInertialWeight Minimum inertial weight for velocity update.
             * @param VelocityFactor Factor for limiting velocity update.
             * @param VelocityConfinement Type of velocity confinement method.
             * @param InitialPositionType Type of initial position distribution.
             * @param TrajectoryType Type of trajectory.
             * @param Log Flag indicating whether to log information during optimization.
             */
            AIPSOPlanner (const APoint &LowerBound, const APoint &UpperBound,
                          int MaximumIteration, int NPopulation, int NBreakpoint, int NWaypoint,
                          double SocialCoefficient = 1.5, double CognitiveCoefficient = 1.5,
                          double MaximumInertialWeight = 0.9, double MinimumInertialWeight = 0.4,
                          double VelocityFactor = 0.5,
                          int VelocityConfinement = VELOCITY_CONFINEMENT::RANDOM_BACK,
                          INITIAL_POSITION_TYPE InitialPositionType = INITIAL_POSITION::DISTRIBUTED,
                          TRAJECTORY_TYPE TrajectoryType = TRAJECTORY::CUBIC_SPLINE,
                          bool Log = true) :
                          ABasePlanner(LowerBound, UpperBound,
                                       MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                       TrajectoryType),
                          SocialCoefficient_(SocialCoefficient),
                          CognitiveCoefficient_(CognitiveCoefficient),
                          MaximumInertialWeight_(MaximumInertialWeight),
                          MinimumInertialWeight_(MinimumInertialWeight),
                          VelocityFactor_(VelocityFactor),
                          VelocityConfinement_(VelocityConfinement),
                          InitialPositionType_(InitialPositionType),
                          Log_(Log)
            {
                std::cout << "[INFO] IPSO Planner instance has been created. " << std::endl;
            }

            /**
             * @brief Destructor.
             */
            ~AIPSOPlanner () override = default;

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

                // Initialize population and velocity bounds.
                this->Population_ = std::vector<AParticle>(this->NPopulation_);
                this->MaximumVelocity_ = (this->UpperBound_ - this->LowerBound_) * this->VelocityFactor_;
                this->MinimumVelocity_ = MaximumVelocity_ * -1.0;

                this->Range_ = this->UpperBound_ - this->LowerBound_;
                this->N_ = static_cast<int>(this->Range_.X * this->Range_.Y);

                // Initialize particles with random positions and velocities.
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; ++PopulationIndex)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    std::vector<APoint> Position(this->NBreakpoint_);
                    std::vector<APoint> Velocity(this->NBreakpoint_);

                    // Generate random positions and velocities within bounds for each breakpoint.
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

                        APoint RandomVelocity;

                        // The initialized velocity is derived from Equation (3.4) of "Standard Particle Swarm Optimisation" by Maurice Clerc.
                        RandomVelocity = (this->LowerBound_ - RandomPosition) +
                                         (this->UpperBound_ - LowerBound_) * GenerateRandom(0.0, 1.0);

                        Position[BreakpointIndex] = RandomPosition;
                        Velocity[BreakpointIndex] = RandomVelocity;
                    }

                    CurrentPopulation->Position = Position;
                    CurrentPopulation->Velocity = Velocity;

                    double Cost = ObjectiveFunction(CurrentPopulation->Position);
                    CurrentPopulation->Cost = Cost;

                    this->AverageCost_ += Cost;

                    // Update personal best position and cost.
                    CurrentPopulation->BestPosition = CurrentPopulation->Position;
                    CurrentPopulation->BestCost = Cost;
                    CurrentPopulation->Feedback = std::vector<APoint>(this->NBreakpoint_);

                    // Update global best position and cost.
                    if (Cost < this->GlobalBestCost_)
                    {
                        this->GlobalBestPosition_ = CurrentPopulation->Position;
                        this->GlobalBestCost_ = Cost;
                    }
                }

                // Compute average cost of the initial population.
                this->AverageCost_ /= this->NPopulation_;

                std::cout << "[INFO] IPSO Planner starts optimizing." << std::endl;

                // Start timing
                auto StartTime = std::chrono::high_resolution_clock::now();

                // Run optimization process for a maximum number of iterations.
                for (int Iteration = 1; Iteration <= this->MaximumIteration_; ++Iteration)
                {
                    // Optimize particle positions and velocities.
                    Optimize(Iteration);

                    this->Convergence_.push_back(this->GlobalBestCost_);

                    // Update average cost for the next iteration.
                    this->NextAverageCost_ /= this->NPopulation_;
                    this->AverageCost_ = this->NextAverageCost_;
                    this->NextAverageCost_ = 0.0;

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

                // Construct the path based on the global best position.
                auto Breakpoint = ConstructBreakpoint(this->GlobalBestPosition_);
                auto X = Breakpoint.first;
                auto Y = Breakpoint.second;

                double Length = 0.0;

                switch (this->TrajectoryType_)
                {
                    case TRAJECTORY::LINEAR:
                        LinearPath(Length, Waypoint, X, Y);
                        break;

                    case TRAJECTORY::CUBIC_SPLINE:
                        CubicSplinePath(Length, Waypoint, X, Y);
                        break;

                    default:
                        CubicSplinePath(Length, Waypoint, X, Y);
                }

                // Check if the path is empty.
                if (Waypoint.empty())
                {
                    std::cerr << "[ERROR] Path not found!" << std::endl;

                    return STATE::FAILED;
                }

                double Error = Penalty(Waypoint) * this->PenaltyScalingFactor_;
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

        protected:
            double MaximumInertialWeight_; /**< Maximum inertial weight for velocity update. */
            double MinimumInertialWeight_; /**< Minimum inertial weight for velocity update. */
            double InertialWeight_{}; /**< Current inertial weight. */
            double SocialCoefficient_; /**< Social coefficient for velocity update. */
            double CognitiveCoefficient_; /**< Cognitive coefficient for velocity update. */
            double VelocityFactor_; /**< Factor for limiting velocity update. */

            std::vector<AParticle> Population_; /**< Vector containing particles. */
            APoint MaximumVelocity_; /**< Vector containing maximum velocity for each breakpoint. */
            APoint MinimumVelocity_; /**< Vector containing minimum velocity for each breakpoint. */

            double AverageCost_ = 0.0; /**< Average cost of the population. */
            double NextAverageCost_ = 0.0; /**< Next average cost of the population. */

            INITIAL_POSITION_TYPE InitialPositionType_; /**< Type of initial position distribution. */
            int VelocityConfinement_; /**< Type of velocity confinement method. */
            bool Log_; /**< Flag indicating whether to log information during optimization. */

        private:
            /**
             * @brief Optimize the IPSO algorithm for the current iteration.
             *
             * This function optimizes the IPSO algorithm for the current iteration.
             * It updates the velocity and position of each particle, evaluates the cost of the updated position,
             * updates the best position of each particle, and updates the global best position.
             *
             * @param Iteration The current iteration.
             *
             * @note A random positive feedback factor is added into the velocity update formula of the particles, which enhances the ability to find a local optimum.
             *       This information is extracted from the paper "A cubic spline method combing improved particle swarm optimization for robot path planning in dynamic uncertain environment"
             *       by Wen Li, Mao Tan, Ling Wang, and Qiuzhen Wang. The feedback update is derived from Equation (7) and (8) of their paper.
             *       Link to the paper: https://journals.sagepub.com/doi/10.1177/1729881419891661
             */
            void Optimize (int Iteration)
            {
                // Iterate over each particle in the population
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; ++PopulationIndex)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    // Calculate adaptive inertial weight for the current particle
                    CalculateAdaptiveInertialWeight(CurrentPopulation);

                    // Update velocity for each dimension of the particle
                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; ++BreakpointIndex)
                    {
                        UpdateVelocity(CurrentPopulation, BreakpointIndex);
                    }

                    // Update position for each dimension of the particle
                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; ++BreakpointIndex)
                    {
                        UpdatePosition(CurrentPopulation, BreakpointIndex);
                    }

                    // Evaluate cost of the updated position for the particle
                    double Cost = ObjectiveFunction(CurrentPopulation->Position);
                    CurrentPopulation->Cost = Cost;

                    // Accumulate cost for calculating average cost
                    this->NextAverageCost_ += Cost;

                    // Update personal best position, cost, and feedback of the particle if applicable
                    if (Cost < CurrentPopulation->BestCost)
                    {
                        CurrentPopulation->BestPosition = CurrentPopulation->Position;
                        CurrentPopulation->BestCost = Cost;

                        for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; ++BreakpointIndex)
                        {
                            // Equation (7)
                            CurrentPopulation->Feedback[BreakpointIndex] = CurrentPopulation->Feedback[BreakpointIndex] * (1.0 / static_cast<double>(Iteration)) +
                                                                           (CurrentPopulation->Velocity[BreakpointIndex]) *
                                                                           GenerateRandom(0.0, 1.0);
                        }
                    } 
                    else 
                    {
                        for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; ++BreakpointIndex)
                        {
                            // Equation (8)
                            CurrentPopulation->Feedback[BreakpointIndex] = CurrentPopulation->Feedback[BreakpointIndex] * (1.0 / static_cast<double>(Iteration)) -
                                                                           (CurrentPopulation->Velocity[BreakpointIndex]) *
                                                                           GenerateRandom(0.0, 1.0);
                        }
                    }

                    // Update global best position and cost if applicable
                    if (Cost < this->GlobalBestCost_)
                    {
                        this->GlobalBestPosition_ = CurrentPopulation->Position;
                        this->GlobalBestCost_ = Cost;
                    }
                }
            }

            /**
             * @brief Calculate the adaptive inertial weight based on the current population's cost.
             *
             * This function calculates the adaptive inertial weight based on the current population's cost,
             * the average cost of the population, and the global best cost.
             * The inertial weight is adjusted based on the difference between the current cost and the global best cost and
             * the difference between the average cost of the population and the global best cost.
             *
             * @param CurrentPopulation Pointer to the current population.
             *
             * @note This adaptive inertia weight adjustment mechanism is inspired by the work of Zhenjian Yang, Ning Li, Yunjie Zhang, and Jin Li,
             *       who introduced adaptive inertia weight to balance the exploration ability.
             *       Specifically, they proposed an adaptive inertia weight approach in their paper "Mobile Robot Path Planning Based on Improved Particle Swarm Optimization and Improved Dynamic Window Approach".
             *       The inertia weight adjustment method described here is derived from Equation (8) of their paper.
             *       Link to the paper: https://www.hindawi.com/journals/jr/2023/6619841/
             */
            void CalculateAdaptiveInertialWeight (AParticle *CurrentPopulation)
            {
                if (CurrentPopulation->Cost <= this->AverageCost_)
                {
                    this->InertialWeight_ = this->MinimumInertialWeight_ +
                                            (this->MaximumInertialWeight_ - this->MinimumInertialWeight_) *
                                            ((CurrentPopulation->Cost - this->GlobalBestCost_) /
                                             (this->AverageCost_ - this->GlobalBestCost_));
                }
                else
                {
                    this->InertialWeight_ = this->MinimumInertialWeight_ +
                                            (this->MaximumInertialWeight_ - this->MinimumInertialWeight_) *
                                            ((this->AverageCost_ - this->GlobalBestCost_) /
                                             (CurrentPopulation->Cost - this->GlobalBestCost_));
                }
            }

            /**
             * @brief Update the velocity of a particle for a specific breakpoint.
             *
             * This function updates the velocity of a particle for a specific breakpoint.
             * It considers the inertial weight, social and cognitive coefficients, as well as a random positive feedback factor
             * to adjust the velocity.
             *
             * @param CurrentPopulation Pointer to the current particle.
             * @param BreakpointIndex The index of the breakpoint for which the velocity is updated.
             */
            void UpdateVelocity (AParticle *CurrentPopulation, int BreakpointIndex)
            {
                APoint NewVelocity;

                // Calculate the new velocity
                NewVelocity.X = this->InertialWeight_ * CurrentPopulation->Velocity[BreakpointIndex].X +
                                this->SocialCoefficient_ * GenerateRandom(0.0, 1.0) *
                                (CurrentPopulation->BestPosition[BreakpointIndex].X -
                                CurrentPopulation->Position[BreakpointIndex].X) +
                                this->CognitiveCoefficient_ * GenerateRandom(0.0, 1.0) *
                                (this->GlobalBestPosition_[BreakpointIndex].X -
                                CurrentPopulation->Position[BreakpointIndex].X) +
                                CurrentPopulation->Feedback[BreakpointIndex].X;
                NewVelocity.Y = this->InertialWeight_ * CurrentPopulation->Velocity[BreakpointIndex].Y +
                                this->SocialCoefficient_ * GenerateRandom(0.0, 1.0) *
                                (CurrentPopulation->BestPosition[BreakpointIndex].Y -
                                CurrentPopulation->Position[BreakpointIndex].Y) +
                                this->CognitiveCoefficient_ * GenerateRandom(0.0, 1.0) *
                                (this->GlobalBestPosition_[BreakpointIndex].Y -
                                CurrentPopulation->Position[BreakpointIndex].Y) +
                                CurrentPopulation->Feedback[BreakpointIndex].Y;

                // Clamp the new velocity to stay within specified bounds
                NewVelocity.X = CLAMP(NewVelocity.X, this->MinimumVelocity_.X, this->MaximumVelocity_.X);
                NewVelocity.Y = CLAMP(NewVelocity.Y, this->MinimumVelocity_.Y, this->MaximumVelocity_.Y);

                // Update velocity
                CurrentPopulation->Velocity[BreakpointIndex] = NewVelocity;
            }

            /**
             * @brief Update the position of a particle for a specific breakpoint.
             *
             * This function updates the position of a particle for a specific breakpoint.
             * It considers the particle's current position and velocity, applies velocity confinement if necessary,
             * and clamps the new position within specified bounds.
             *
             * @param CurrentPopulation Pointer to the current particle.
             * @param BreakpointIndex The index of the breakpoint for which the position is updated.
             */
            void UpdatePosition (AParticle *CurrentPopulation, int BreakpointIndex) const
            {
                // Calculate the temporary new position by adding the updated velocity to the current position
                APoint TemporaryNewPosition = CurrentPopulation->Position[BreakpointIndex] + 
                                              CurrentPopulation->Velocity[BreakpointIndex];

                // Apply velocity confinement if the temporary new position is out of bounds
                if (IS_OUT_OF_BOUND(TemporaryNewPosition.X, this->LowerBound_.X, this->UpperBound_.X) ||
                    IS_OUT_OF_BOUND(TemporaryNewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y)) 
                {
                    APoint Confinement;

                    switch (this->VelocityConfinement_)
                    {
                        case VELOCITY_CONFINEMENT::RANDOM_BACK:
                            Confinement = RandomBackConfinement(CurrentPopulation->Velocity[BreakpointIndex]);
                            break;

                        case VELOCITY_CONFINEMENT::HYPERBOLIC:
                            Confinement = HyperbolicConfinement(CurrentPopulation->Position[BreakpointIndex],
                                                                CurrentPopulation->Velocity[BreakpointIndex]);
                            break;

                        case VELOCITY_CONFINEMENT::MIXED:
                            Confinement = MixedConfinement(CurrentPopulation->Position[BreakpointIndex],
                                                           CurrentPopulation->Velocity[BreakpointIndex]);
                            break;

                        default:
                            Confinement = RandomBackConfinement(CurrentPopulation->Velocity[BreakpointIndex]);
                    }

                    // Apply the confinement to the velocity
                    CurrentPopulation->Velocity[BreakpointIndex] = Confinement;
                }

                // Calculate the new position by adding the updated velocity to the current position
                APoint NewPosition = CurrentPopulation->Position[BreakpointIndex] + 
                                     CurrentPopulation->Velocity[BreakpointIndex];

                // Clamp the new position to stay within specified bounds
                NewPosition.X = CLAMP(NewPosition.X, this->LowerBound_.X, this->UpperBound_.X);
                NewPosition.Y = CLAMP(NewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y);

                // Update position
                CurrentPopulation->Position[BreakpointIndex] = NewPosition;
            }

            /**
             * @brief Apply random back velocity confinement.
             *
             * This function applies random back velocity confinement to a given velocity.
             * It generates a random factor within the range [0, 1] and multiplies it with the given velocity.
             *
             * @param Velocity The velocity to which random back confinement is applied.
             *
             * @return The velocity after applying random back confinement.
             */
            static APoint RandomBackConfinement (const APoint &Velocity)
            {
                APoint VelocityConfinement = Velocity * -GenerateRandom(0.0, 1.0);

                return VelocityConfinement;
            }

            /**
             * @brief Apply hyperbolic velocity confinement.
             *
             * This function applies hyperbolic velocity confinement to a given velocity.
             * It calculates the velocity confinement based on the position and velocity of the particle,
             * as well as the specified lower and upper bounds for the position.
             *
             * @param LowerBound The lower bound for the position (search space).
             * @param UpperBound The upper bound for the position (search space).
             * @param Position The current position of the particle.
             * @param Velocity The velocity to which hyperbolic confinement is applied.
             *
             * @return The velocity after applying hyperbolic confinement.
             */
            APoint HyperbolicConfinement (const APoint &Position, const APoint &Velocity) const
            {
                APoint VelocityConfinement;

                if (Velocity.X > 0.0)
                {
                    VelocityConfinement.X = Velocity.X /
                                            (1.0 + abs(Velocity.X / (this->UpperBound_.X - Position.X)));
                }
                else
                {
                    VelocityConfinement.X = Velocity.X /
                                            (1.0 + abs(Velocity.X / (Position.X - this->LowerBound_.X)));
                }

                if (Velocity.Y > 0.0)
                {
                    VelocityConfinement.Y = Velocity.Y /
                                            (1.0 + abs(Velocity.Y / (this->UpperBound_.Y - Position.Y)));
                }
                else
                {
                    VelocityConfinement.Y = Velocity.Y /
                                            (1.0 + abs(Velocity.Y / (Position.Y - this->LowerBound_.Y)));
                }

                return VelocityConfinement;
            }

            /**
             * @brief Apply mixed velocity confinement.
             *
             * This function applies mixed velocity confinement to a given velocity.
             * It randomly selects between hyperbolic and random back velocity confinement based on a generated random factor.
             *
             * @param LowerBound The lower bound for the position (search space).
             * @param UpperBound The upper bound for the position (search space).
             * @param Position The current position of the particle.
             * @param Velocity The velocity to which mixed confinement is applied.
             *
             * @return The velocity after applying mixed confinement.
             */
            APoint MixedConfinement (const APoint &Position, const APoint &Velocity) const
            {
                APoint VelocityConfinement;

                if (GenerateRandom(0.0, 1.0) >= 0.5) {
                    VelocityConfinement = HyperbolicConfinement(Position, Velocity);
                }
                else
                {
                    VelocityConfinement = RandomBackConfinement(Velocity);
                }

                return VelocityConfinement;
            }
        };
    } // IPSO
} // MTH

#endif // IPSO_PLANNER_H
