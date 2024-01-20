//
// Created by Sippawit Thammawiset on 13/1/2024 AD.
//

#include <iostream>
#include <fstream>
#include <sstream>

#include <PSO.h>

//PSO_Planner::APoint LowerBound(-2, -6);
//PSO_Planner::APoint UpperBound(10, 6);

//PSO_Planner::APoint LowerBound(143, 127);
//PSO_Planner::APoint UpperBound(257, 266);

PSO_Planner::APoint LowerBound(0, 0);
PSO_Planner::APoint UpperBound(384, 384);

PSO_Planner::APoint Start(214, 159);
PSO_Planner::APoint Goal(181, 240);

int MaxIteration = 5000;
int Population = 15;
int Breakpoint = 3;
int InterpolationPoint = 25;
int Waypoint = 1 + (Breakpoint + 1) * InterpolationPoint;

std::vector<PSO_Planner::APoint> Path;

void ReadCostmap (const std::string& File, unsigned char *Costmap, int Row, int Column) {
    std::ifstream file(File);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << File << std::endl;
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < Row; ++i) {
        std::string line;
        if (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;
            for (int j = 0; j < Column; ++j) {
                if (std::getline(iss, token, '\t')) {
                    Costmap[i * Column + j] = static_cast<unsigned char>(std::stoi(token));
                } else {
                    std::cerr << "Error reading data from file: " << File << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        } else {
            std::cerr << "Error reading line from file: " << File << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    file.close();
}

int main ()
{
    std::string File = "../out4.txt";
    int N = 384; // Number of Row
    int M = 384; // Number of columns

    auto* Costmap = new unsigned char[N * M];

    ReadCostmap(File, Costmap, N, M);

//    for (int i = 0; i < N; ++i) {
//        for (int j = 0; j < M; ++j) {
//            std::cout << static_cast<int>(Costmap[i * M + j]) << "\t";
//        }
//        std::cout << std::endl;
//    }

    PSO_Planner::APSO PSO(LowerBound, UpperBound,
                          1.0f, 0.0f, 0.0f,
                          MaxIteration, Population, Breakpoint, Waypoint,
                          0.9f, 1.5f, 1.5f,
                          0.5f, 1.00f
                          );

    if (PSO.CreatePlan(Costmap, Start, Goal, Path))
    {
        for (PSO_Planner::APoint &i : Path)
        {
            std::cout << i.X << "\t" << i.Y << "\n";
        }
    }

    return 0;
}