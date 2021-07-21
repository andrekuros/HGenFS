#pragma once

#include <iostream>
#include <sstream>
#include <vector>
#include <array>
#include <random>
#include <string>
#include <algorithm>
#include <chrono>


class Individual
{

public:

	Individual(std::vector<std::vector<double>>* timeTruck, std::vector<std::vector<double>>* timeDrone, double droneEndurance, int dronesAvailable, int size, double seed);

	//Params that define the Individual Solution
	std::vector<int> solution; //General Path
	std::vector<int> means; //0 for truck and 1 for drone (for a while)	

	std::vector<std::vector<int>> dronePath; //Constumer served by drone + dep and land points	
	//std::vector<bool> hasDep; 

	//Configurations
	double droneEndurance;
	int dronesAvailable;
	int size;
	bool droneFromDepot = false;
	double penaultyFactor = 15;

	std::pair<std::vector<int>, std::vector<std::vector<int>>> code;

	//Evaluations	
	double fit{ -1.0 };
	double objFunction{ -1.0 };
	double penaulty = 0.0;
	double syncLoss = 0.0;
	double truckTime = 0.0;
	std::pair<double, double> knownBefore = { -1, 0 };

	bool CreateDronePath(bool dronePathLocalSearch , bool checkDroneEndurance , bool checkSync = false);
	std::pair<double, double>  getSyncDiff(int solIdx);
	//void genCode();
	void printSol();
	std::string getSolutionCsv();

	std::mt19937 rndGen;
	std::vector<std::vector<double>>* timeTruck;
	std::vector<std::vector<double>>* timeDrone;

};
