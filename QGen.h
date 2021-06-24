﻿#pragma once

#include <random>
#include <map>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include "individual.h"


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

	QGen(int populationSize, double mutationRate, double csRate, int numVariables, int varMin, int varMax, int seed);

	void genFirstPopulation(std::string option);
	void processPopulation(std::string problemType);

	void newGeneration();
	
	int populationSize;
	double mtRate;
	int csRate;
	int elite;
	int best;
	double varMax;
	double varMin;
	int numVariables;
	std::string init;
	int seed;
	double truckProportion;
	double droneEndurance;

	std::random_device rd;
	std::mt19937 rndGen;

	std::vector<Individual*>* newPop;
	std::vector<Individual*>* evaluatedPop;
	
	std::vector<std::vector<double>>* timeTruck;
	std::vector<std::vector<double>>* timeDrone;
	std::vector<node> nodes;
	double droneSpeed{ -1 };

};