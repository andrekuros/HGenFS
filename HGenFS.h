#pragma once

#include <random>
#include <map>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include "individual.h"
#include <thread>        
#include <future>
#include <mutex>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <set>

std::mutex mtx;
double bestAll = 9999;

std::vector<std::string> bestList;
std::vector<std::string> evolList;
class node
{
public:
	std::string type;
	double x;
	double y;
	int hasDroneAcess;
};

bool printResults;
bool saveGenerations;
bool saveMain;
bool saveEvolution;
int indsPrint;
double stopTime;

std::string resultsPath = ""; // "~/share/HGenFS/HGenFS/results/";
//std::string pathProblems = "problems/FSTSP_10_customer_problems/20140810T123";
std::string pathProblems = "problems/ponza16/uniform/";
std::string problem;


class HGenFS
{
public:

	HGenFS(int populationSize, int numVariables, int varMin, int varMax, int seed, std::string problem, std::string problemPath);
	~HGenFS();
	void genFirstPopulation(std::string option);
	void processPopulation(std::string problemType);
	std::pair<double, double> fitFromSolutionTable(Individual* ind);

	void childrenFromCrossOver(int ind1, int ind2, std::string type, std::string selectionType, std::vector<int>* usedList);

	void newGeneration();
	void processInd(Individual* ind);
	void localSearch(Individual* ind, int ns, int nd, bool checkPath, bool checkEndurance);
	
	Individual* selection(std::string method, Individual* anotherParent ,std::vector<int>* usedList);

	double best = 999999;
	int bestIn = 0;
	double bestTruck = 9999999;

	int populationSize;
	double mtRate{ 0.05 };
	double csProportion{ 0.70 };
	int csSize = (int)(populationSize * csProportion);
	
	bool dronePathSyncSearch = false;
	int localSearchNum[2] = { 0,0 };
	bool localSearchParams[2] = { false, false };
	bool pathCheckEndurance = true;

	
	double csSearchTimes = 50;	

	double eliteProportion{ 0.01 };
	double eliteSize = (int)(populationSize * eliteProportion);
		
	double varMax;
	double varMin;
	int numVariables;

	double tournamentP = 0.5;
	double tournamentProportion = 0.60;
	int tournamentSize;
	std::vector<int> tournamentGroup;
	double fitRange {0};
	
	std::string init;
	int seed;

	//Dataset Definitions
	std::string problemPath;
	std::string problem;
	std::string dataset{ "murray" };
	double truckProportion;
	double droneEndurance{ 1000000 };
	int dronesAvailable{ 1 };
	double truckSpeed{ 1.0 };
	double droneSpeed{ 1.0 };
	double sl{ 0 };
	double sr{ 0 };

	std::random_device rd;
	std::mt19937 rndGen;
	int lastKnown = 0;

	std::vector<Individual*>* newPop;
	std::vector<Individual*>* evaluatedPop;

	std::vector<std::vector<double>>* timeTruck;
	std::vector<std::vector<double>>* timeDrone;
	std::vector<node> nodes;	

};
