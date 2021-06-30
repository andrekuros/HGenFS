#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <random>

class Individual
{

public:
	
	Individual(std::vector<std::vector<double>>* timeTruck, std::vector<std::vector<double>>* timeDrone, double droneEndurance, int dronesAvailable, int size, double seed);
	std::vector<int> solution;
	std::vector<int> means;
	std::vector<std::vector<int>> dronePath;
	//std::vector<int> means; //0 for truck and 1 for drone (for a while)
	double fit{ -1 };
	double objFunction{ -1 };
	double droneEndurance;
	int dronesAvailable;
	int size;
	bool CreateDronePath();
	double getSyncDiff(int solIdx);

	std::mt19937 rndGen;
	std::vector<std::vector<double>>* timeTruck;
	std::vector<std::vector<double>>* timeDrone;

};
