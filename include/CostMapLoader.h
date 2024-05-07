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
        /**
         * @brief An enum defining of maps.
         */
        enum MAP
        {
            VALIDATED = 0,            /**< The validated map is used to validate path planning algorithms.
                                        * All algorithms should output a straight line between the start and goal points.
                                        */
            SCENARIO_1 = 1,           /**< Scenario 1 map. */
            SCENARIO_2 = 2,           /**< Scenario 2 map. */
            SCENARIO_3 = 3,           /**< Scenario 3 map. */
            TURTLEBOT3_WORLD = 4,     /**< TurtleBot3 world map. */
            // Add more maps here
        };
    }

    /**
     * @brief Read the cost map from a file and store it in a buffer.
     *
     * @param File Path to the file containing the cost map.
     * @param CostMap Pointer to the buffer to store the cost map.
     * @param Width Width of the cost map.
     * @param Height Height of the cost map.
     */
    void ReadCostMap (const std::string &File, unsigned char *CostMap, int Width, int Height)
    {
        std::ifstream Reader(File);

        // Check if the file is successfully opened
        if (!Reader.is_open())
        {
            std::cerr << "[ERROR] Error opening cost map file: " << File << std::endl;
            exit(EXIT_FAILURE);
        }

        // Read each line of the file
        for (int j = 0; j < Height; ++j)
        {
            std::string Line;

            // Get the line
            if (std::getline(Reader, Line))
            {
                std::istringstream SS(Line);
                std::string Token;

                // Parse tokens separated by tab character
                for (int i = 0; i < Width; ++i)
                {
                    if (std::getline(SS, Token, '\t'))
                    {
                        CostMap[j * Width + i] = static_cast<unsigned char>(std::stoi(Token));
                    }
                    else
                    {
                        std::cerr << "[ERROR] Error reading cost map from file: " << File << std::endl;
                        exit(EXIT_FAILURE);
                    }
                }
            }
            else
            {
                std::cerr << "[ERROR] Error reading line from cost map file: " << File << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        // Close the file
        Reader.close();
    }

    /**
     * @brief Display the cost map to the console.
     *
     * @param CostMap Pointer to the buffer containing the cost map.
     * @param Width Width of the cost map.
     * @param Height Height of the cost map.
     */
    void DisplayCostMap (const unsigned char *CostMap, int Width, int Height)
    {
        for (int y = 0; y < Height; ++y)
        {
            for (int x = 0; x < Width; ++x)
            {
                std::cout << static_cast<int>(CostMap[y * Width + x]) << "\t";
            }
            std::cout << std::endl;
        }
    }

    /**
     * @brief Load a predefined cost map based on the specified COST_MAP_NAME.
     *
     * @param Width Output parameter to store the width of the loaded cost map.
     * @param Height Output parameter to store the height of the loaded cost map.
     * @param CostMapName Enum specifying the predefined map to load.
     * @param Start Output parameter to store the start point of the path.
     * @param Goal Output parameter to store the goal point of the path.
     *
     * @return Pointer to the loaded cost map.
     */
    unsigned char * CostMapLoader (COST_MAP_NAME CostMapName,
                                   int &Width, int &Height,
                                   MTH::APoint &Start, MTH::APoint &Goal)
    {
        std::string CostMapFile;

        switch (CostMapName)
        {
            case MAP::VALIDATED:
                CostMapFile = "../map/validated.txt";
                Width = 256;
                Height = 256;
                Start = MTH::APoint(44, 205);
                Goal = MTH::APoint(205, 44);
                break;

            case MAP::SCENARIO_1:
                CostMapFile = "../map/scenario1.txt";
                Width = 2500;
                Height = 1250;
                Start = MTH::APoint(2192, 658);
                Goal = MTH::APoint(292, 949);
                break;

            case MAP::SCENARIO_2:
                CostMapFile = "../map/scenario2.txt";
                Width = 2500;
                Height = 2291;
                Start = MTH::APoint(718, 1837);
                Goal = MTH::APoint(1589, 1214);
                break;

            case MAP::SCENARIO_3:
                CostMapFile = "../map/scenario3.txt";
                Width = 384;
                Height = 384;
                Start = MTH::APoint(41, 353);
                Goal = MTH::APoint(346, 26);
                break;

            case MAP::TURTLEBOT3_WORLD:
                CostMapFile = "../map/turtlebot3_world.txt";
                Width = 384;
                Height = 384;
                Start = MTH::APoint(211, 235);
                Goal = MTH::APoint(171, 176);
                break;

                // Add more cases for additional predefined maps

            default:
                CostMapFile = "../map/turtlebot3_world.txt";
                Width = 384;
                Height = 384;
                Start = MTH::APoint(211, 235);
                Goal = MTH::APoint(171, 176);
        }

        auto *CostMap = new unsigned char[Width * Height];

        // Read the cost map from file
        ReadCostMap(CostMapFile, CostMap, Width, Height);

        return CostMap;
    }
} // CostMapLoader

#endif // COST_MAP_LOADER_H
