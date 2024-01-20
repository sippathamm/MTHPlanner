//
// Created by Sippawit Thammawiset on 20/1/2024 AD.
//

#include <iostream>
#include <fstream>
#include <sstream>

// Function to read data from a text file and store in a 1D array
void readDataFromFile(const std::string& filename, unsigned char* dataArray, int rows, int cols) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < rows; ++i) {
        std::string line;
        if (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;
            for (int j = 0; j < cols; ++j) {
                if (std::getline(iss, token, '\t')) {
                    dataArray[i * cols + j] = static_cast<unsigned char>(std::stoi(token));
                } else {
                    std::cerr << "Error reading data from file: " << filename << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        } else {
            std::cerr << "Error reading line from file: " << filename << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    file.close();
}

int main() {
    // Specify the filename and array dimensions (N and M)
    std::string filename = "../out4.txt";
    int N = 384; // Number of rows
    int M = 384; // Number of columns

    // Allocate memory for the 1D array
    unsigned char* dataArray = new unsigned char[N * M];

    // Read data from file and store in the 1D array
    readDataFromFile(filename, dataArray, N, M);

    // Display the contents of the array
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            std::cout << static_cast<int>(dataArray[i * M + j]) << "\t";
        }
        std::cout << std::endl;
    }

    std::cout << (int)dataArray[(193 - 1) * M + (166 - 1)] << "\n";

    // Deallocate memory
    delete[] dataArray;

    return 0;
}
