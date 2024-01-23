//
// Created by Sippawit Thammawiset on 13/1/2024 AD.
//

#include <iostream>
#include <fstream>
#include <sstream>

#include <PSO.h>

//PSO_Planner::APoint LowerBound(-2, -6);
//PSO_Planner::APoint UpperBound(10, 6);

PSO_Planner::APoint LowerBound(0, 0);
PSO_Planner::APoint UpperBound(384, 384);

PSO_Planner::APoint Start(214, 159);
PSO_Planner::APoint Goal(181, 240);

int MaxIteration = 5000;
int Population = 15;
int Breakpoint = 3;
int InterpolationPoint = 25;
int Waypoint = 1 + (Breakpoint + 1) * InterpolationPoint;

double InertialCoefficient = 0.9f;
double SocialCoefficient = 1.5f;
double CognitiveCoefficient = 1.5f;

double VelocityFactor = 0.5f;
double ObstacleCostFactor = 0.95f;

std::vector<PSO_Planner::APoint> Path;

void ReadCostmap (const std::string& File, unsigned char *Costmap, int X, int Y);
void DisplayCostmap (const unsigned char *Costmap, int X, int Y);

int main ()
{
    std::string CostmapFile = "../Costmap.txt";
    int X = 384;
    int Y = 384;

    auto* Costmap = new unsigned char[X * Y];

    ReadCostmap(CostmapFile, Costmap, X, Y);
//    DisplayCostmap(Costmap, X, Y);

    PSO_Planner::APSO PSO(LowerBound, UpperBound,
                          MaxIteration, Population, Breakpoint, Waypoint,
                          InertialCoefficient, SocialCoefficient, CognitiveCoefficient,
                          VelocityFactor, ObstacleCostFactor
                          );

    if (PSO.CreatePlan(Costmap, Start, Goal, Path))
    {
        std::cout << "Waypoint = [";
        for (PSO_Planner::APoint &i : Path)
        {
            std::cout << "(" << i.X << ", " << i.Y << ")" << ", ";
        }
        std::cout << "]";
    }

    delete[] Costmap;

    return 0;
}

void ReadCostmap (const std::string& File, unsigned char *Costmap, int X, int Y) {
    std::ifstream ReadFile(File);

    if (!ReadFile.is_open()) {
        std::cerr << "Error opening costmap file: " << File << std::endl;
        exit(EXIT_FAILURE);
    }

    for (int j = 0; j < Y; j++)
    {
        std::string Line;

        if (std::getline(ReadFile, Line))
        {
            std::istringstream _(Line);
            std::string Token;

            for (int i = 0; i < X; i++)
            {
                if (std::getline(_, Token, '\t'))
                {
                    Costmap[j * X + i] = static_cast<unsigned char>(std::stoi(Token));
                }
                else
                {
                    std::cerr << "Error reading constmap from file: " << File << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }
        else
        {
            std::cerr << "Error reading line from costmap file: " << File << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    ReadFile.close();
}

void DisplayCostmap (const unsigned char *Costmap, int X, int Y)
{
    for (int j = 0; j < Y; j++)
    {
        for (int i = 0; i < X; i++)
        {
            std::cout << static_cast<int>(Costmap[j * Y + i]) << "\t";
        }
        std::cout << std::endl;
    }
}
