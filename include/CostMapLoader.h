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

    namespace MAP
    {
        enum MAP
        {
            SCENARIO_1 = 1,
            SCENARIO_2 = 2,
            TURTLEBOT3_WORLD = 3,
            // Define your cost map name with ID here
        };
    }

    void ReadCostMap (const std::string& File, unsigned char *CostMap, int Width, int Height)
    {
        std::ifstream Reader(File);

        if (!Reader.is_open())
        {
            std::cerr << "Error opening cost map file: " << File << std::endl;
            exit(EXIT_FAILURE);
        }

        for (int j = 0; j < Height; ++j)
        {
            std::string Line;

            if (std::getline(Reader, Line))
            {
                std::istringstream _(Line);
                std::string Token;

                for (int i = 0; i < Width; ++i)
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

        Reader.close();
    }

    void DisplayCostMap (const unsigned char *CostMap, int Width, int Height)
    {
        for (int i = 0; i < Width; ++i)
        {
            for (int j = 0; j < Height; ++j)
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
            case MAP::SCENARIO_1:
                CostMapFile = "../map/scenario1.txt";

                Width = 2500;
                Height = 1250;

                Start = MTH::APoint (2192, 658);
                Goal = MTH::APoint (292, 949);

                break;

            case MAP::SCENARIO_2:
                CostMapFile = "../map/scenario2.txt";

                Width = 2500;
                Height = 2291;

                Start = MTH::APoint (718, 1837);
                Goal = MTH::APoint (1589, 1214);

                break;

            case MAP::TURTLEBOT3_WORLD:
                CostMapFile = "../map/turtlebot3_world.txt";

                Width = 384;
                Height = 384;

                Start = MTH::APoint (211, 235);
                Goal = MTH::APoint (171, 176);

                break;

            // Define your cost map properties and start and goal here

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
} // CostMapLoader

#endif // COST_MAP_LOADER_H
