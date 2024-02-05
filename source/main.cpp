//
// Created by Sippawit Thammawiset on 3/2/2024 AD.
//

#include "PSOPlanner.h"

#include <fstream>
#include <sstream>

void ReadCostMap (const std::string& File, unsigned char *CostMap, int Width, int Height);
void DisplayCostMap (const unsigned char *CostMap, int Width, int Height);

int main ()
{
//    std::string CostMapFile = "../turtlebot3_world.txt";
//    const int Width = 384;
//    const int Height = 384;
//
//    Optimizer::APoint Start(214, 159);
//    Optimizer::APoint Goal(181, 240);

//    std::string CostMapFile = "../environment1.txt";
//    const int Width = 384;
//    const int Height = 384;
//
//    Optimizer::APoint Start(82, 302);
//    Optimizer::APoint Goal(287, 122);

    std::string CostMapFile = "../map/environment2.txt";
    const int Width = 3651;
    const int Height = 3080;

    Optimizer::APoint Start(1000, 2462);
    Optimizer::APoint Goal(2834, 798);

    auto* CostMap = new unsigned char[Width * Height];

    ReadCostMap(CostMapFile, CostMap, Width, Height);
//    DisplayCostMap(CostMap, Width, Height);

    // PSO Parameter settings
    int MaximumIteration = 1000;
    int NPopulation = 50;
    int NBreakpoint = 5;
    int NInterpolationPoint = 30;
    int NWaypoint = 1 + (NBreakpoint + 1) * NInterpolationPoint;

    double SocialCoefficient = 2.0f;
    double CognitiveCoefficient = 1.2f;
    double VelocityFactor = 0.5;

    Optimizer::APoint LowerBound(0.0f, 0.0f);
    Optimizer::APoint UpperBound(Width, Height);

    Optimizer::AGlobalPlanner Planner(LowerBound, UpperBound,
                                      MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                      SocialCoefficient, CognitiveCoefficient,
                                      VelocityFactor,
                                      Optimizer::CUBIC_SPLINE);

    std::vector<Optimizer::APoint> Path;

    auto start = std::chrono::high_resolution_clock::now();
    if (Planner.CreatePlan(CostMap, Start, Goal, Path))
    {
        auto stop = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

        std::cout << "Waypoint: [";
        for (const auto &i : Path)
        {
            std::cout << "(" << i.X << ", " << i.Y << ")" << ", ";
        }
        std::cout << "]";

        std::cout << std::endl;

        std::cout << "Length: " << Planner.GetPathLength() << std::endl;
        std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;
    }

    delete[] CostMap;

    return 0;
}

void ReadCostMap (const std::string& File, unsigned char *CostMap, int Width, int Height) {
    std::ifstream ReadFile(File);

    if (!ReadFile.is_open()) {
        std::cerr << "Error opening cost map file: " << File << std::endl;
        exit(EXIT_FAILURE);
    }

    for (int j = 0; j < Height; j++)
    {
        std::string Line;

        if (std::getline(ReadFile, Line))
        {
            std::istringstream _(Line);
            std::string Token;

            for (int i = 0; i < Width; i++)
            {
                if (std::getline(_, Token, '\t'))
                {
                    CostMap[j * Width + i] = static_cast<unsigned char>(std::stoi(Token));
                }
                else
                {
                    std::cerr << "Error reading cost map from file: " << File << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }
        else
        {
            std::cerr << "Error reading line from cost map file: " << File << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    ReadFile.close();
}

void DisplayCostMap (const unsigned char *CostMap, int Width, int Height)
{
    for (int i = 0; i < Width; i++)
    {
        for (int j = 0; j < Height; j++)
        {
            std::cout << static_cast<int>(CostMap[j * Width + i]) << "\t";
        }
        std::cout << std::endl;
    }
}
