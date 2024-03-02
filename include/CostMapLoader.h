//
// Created by Sippawit Thammawiset on 6/2/2024 AD.
//

#ifndef COST_MAP_LOADER_H
#define COST_MAP_LOADER_H

#include "Point.h"

#include <iostream>
#include <fstream>
#include <sstream>

namespace CostMapLoader
{
    typedef int COST_MAP_NAME;

    enum
    {
        SCENARIO_1 = 1,
        SCENARIO_2 = 2,
        SCENARIO_3 = 3,
        TURTLEBOT3_WORLD = 4,
    };

    void ReadCostMap (const std::string& File, unsigned char *CostMap, int Width, int Height)
    {
        std::ifstream ReadFile(File);

        if (!ReadFile.is_open())
        {
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

    unsigned char *CostMapLoader (int &Width, int &Height, COST_MAP_NAME CostMapName,
                                  MTH::APoint &Start, MTH::APoint &Goal)
    {
        std::string CostMapFile;

        switch (CostMapName)
        {
            case SCENARIO_1:
                CostMapFile = "../map/scenario1.txt";

                Width = 2000;
                Height = 3000;

                Start = MTH::APoint (404, 2402);
                Goal = MTH::APoint (1722, 752);

                break;

            case SCENARIO_2:
                CostMapFile = "../map/scenario2.txt";

                Width = 2000;
                Height = 3000;

                Start = MTH::APoint (250, 300);
                Goal = MTH::APoint (1273, 2726);

                break;

            case SCENARIO_3:
                CostMapFile = "../map/scenario3.txt";

                Width = 3000;
                Height = 2000;

                Start = MTH::APoint (400, 1850);
                Goal = MTH::APoint (2444, 232);

                break;

            case TURTLEBOT3_WORLD:
                CostMapFile = "../map/turtlebot3_world.txt";

                Width = 384;
                Height = 384;

                Start = MTH::APoint (211, 235);
                Goal = MTH::APoint (171, 176);

                break;

            default:
                CostMapFile = "../map/scenario2.txt";

                Width = 2000;
                Height = 3000;

                Start = MTH::APoint (404, 2402);
                Goal = MTH::APoint (1722, 752);
        }

        auto *CostMap = new unsigned char [Width * Height];

        ReadCostMap(CostMapFile, CostMap, Width, Height);

        return CostMap;
    }

}

#endif // COST_MAP_LOADER_H
