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

std::mutex mtx;

class node
{
public:	
	std::string type;
	double x;
	double y;
	bool hasDroneAcess ;
};


class QGen
{
public:

	QGen(int populationSize, int numVariables, int varMin, int varMax, int seed, std::string problemPath);
	~QGen();
	void genFirstPopulation(std::string option);
	void processPopulation(std::string problemType);
	std::pair<double, double> fitFromSolutionTable(std::string cod);
	
	void childrenFromCrossOver( int ind1, int ind2, std::string type = "swap" );

	void newGeneration();
	Individual* selection(std::string method, Individual* anotherParent = nullptr);
	
	int populationSize;
	double mtRate;
	double csProportion, eliteProportion;
	int csSize, eliteSize, csSearchRate;

	int best;
	double varMax;
	double varMin;
	int numVariables;
		
	double tournamentP = 0.5;
	double tournamentProportion = 0.7;	
	int tournamentSize;

	std::vector<int> tournamentGroup;

	std::unordered_map<std::string, std::pair<double,double>> solutionTable;

	std::string init;
	int seed;
	std::string problemPath;
	double truckProportion;
	double droneEndurance;
	int dronesAvailable;

	std::random_device rd;
	std::mt19937 rndGen;
	int lastKnown = 0;

	std::vector<Individual*>* newPop;
	std::vector<Individual*>* evaluatedPop;
	
	std::vector<std::vector<double>>* timeTruck;
	std::vector<std::vector<double>>* timeDrone;
	std::vector<node> nodes;
	double droneSpeed{ -1 };

};
