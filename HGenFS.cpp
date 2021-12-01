#include "HGenFS.h"

using namespace std;

HGenFS::HGenFS(int populationSize, int numVariables, int varMin, int varMax, int seed, std::string problem, std::string problemPath) :
	populationSize(populationSize), numVariables(numVariables),
	varMin(varMin), varMax(varMax), seed(seed), problemPath(problemPath), problem(problem)
{		
	


	nodes.resize(numVariables);
	newPop = new std::vector<Individual*>;
	evaluatedPop = new std::vector<Individual*>;
	newPop->resize(populationSize);
	evaluatedPop->resize(populationSize);

	if (eliteSize < 5) eliteSize = 5;

	timeTruck = new std::vector<std::vector<double>>;
	timeDrone = new std::vector<std::vector<double>>;

	for (auto& ind : *newPop)
		ind = new Individual(timeTruck, timeDrone, droneEndurance, sr,sl, dronesAvailable, numVariables, rndGen());

	for (auto& ind : *evaluatedPop)
		ind = new Individual(timeTruck, timeDrone, droneEndurance, sr,sl, dronesAvailable, numVariables, rndGen());

	for (int i = 0; i < populationSize; i++)
	{
		tournamentGroup.push_back(i);
	}
	tournamentSize = (int)(populationSize * tournamentProportion);


	if (dataset == "murray")
	{
		//LOAD Costs Truck	
		std::ifstream dataTruck(problemPath + problem + "/tau.csv");
		if (dataTruck.is_open())
		{
			std::string line;
			while (std::getline(dataTruck, line))
			{
				std::string delimiter = ",";
				size_t pos = 0;
				std::string data;

				std::vector<double> row;
				while ((pos = line.find(delimiter)) != std::string::npos)
				{
					data = line.substr(0, pos);
					row.emplace_back(std::stod(data));
					line.erase(0, pos + delimiter.length());
				}
				row.emplace_back(std::stod(line));
				timeTruck->emplace_back(row);
			}
		}
		else
		{
			std::cout << "Wrong File TrackCost: " << problemPath + problem + "/tau.csv";
		}

		//LOAD Costs Drone	
		std::ifstream dataDrone(problemPath + problem + "/tauprime.csv");
		if (dataDrone.is_open())
		{
			std::string line;
			while (std::getline(dataDrone, line))
			{
				std::string delimiter = ",";
				size_t pos = 0;
				std::string data;

				std::vector<double> row;
				while ((pos = line.find(delimiter)) != std::string::npos)
				{
					data = line.substr(0, pos);
					row.emplace_back(std::stod(data));
					line.erase(0, pos + delimiter.length());
				}
				row.emplace_back(std::stod(line));
				timeDrone->emplace_back(row);
			}
		}

		//LOAD Nodes and Drone data	
		std::ifstream dataNodes(problemPath + problem + "/nodes.csv");
		if (dataNodes.is_open())
		{
			std::string line;
			while (std::getline(dataNodes, line))
			{
				std::string delimiter = ",";
				size_t pos = 0;
				std::string data;

				std::vector<double> row;

				while ((pos = line.find(delimiter)) != std::string::npos)
				{
					data = line.substr(0, pos);
					row.emplace_back(std::stod(data));
					line.erase(0, pos + delimiter.length());
				}
				row.emplace_back(std::stod(line));

				int n = (int)(row[0]);
				if (droneSpeed == -1)
				{
					droneSpeed = row[3];
					nodes[n].hasDroneAcess = 0;
				}
				else
				{
					nodes[n].hasDroneAcess = row[3] < 0.9 ? 1 : 0;

				}

				nodes[n].x = row[1];
				nodes[n].y = row[2];
			}
			//nodes.back().hasDroneAcess = false;
			nodes[numVariables - 1].hasDroneAcess = 0;			
		}
	}
	else if (dataset == "ponza")
	{
		sl = 0.6;
		sr = 0.5;
		droneEndurance = 24.0;		
		truckSpeed = 56.32;
		droneSpeed = 80.47;


	
	}
	else {
		std::cout << "\nUnknown dataset";
		exit(-1);
	}
};

/*std::pair<double, double> HGenFS::fitFromSolutionTable(Individual* ind)
{
	auto search = solutionTable.find(ind->code);
	return search == solutionTable.end() ? std::make_pair(-1.0, 0.0) : search->second;
}*/

void HGenFS::genFirstPopulation(std::string option)
{
	init = option;
	rndGen.seed(seed);

	if (init == "random")
	{
		for (auto& ind : *newPop)
		{
			for (int i = 0; i < numVariables; i++)
			{
				ind->solution[i] = i; //soltest[i];//
				ind->means[i] = (unsigned int)(rndGen() % 2);// *nodes[i].hasDroneAcess;			
			}			
			ind->means[numVariables - 1] = 0;
			ind->means[0] = 0;

			
			//Shuffle the sequence of the main path 
			std::shuffle(ind->solution.begin() + 1, ind->solution.end() - 1, rndGen);
			//ind->solution = { 0, 4, 8, 2, 9, 10, 3, 1, 7, 5, 6, 11 };
			ind->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
		}

		for (auto& ind : *evaluatedPop)
		{
			for (int i = 0; i < numVariables; i++)
			{
				ind->solution[i] = i; //soltest[i];//
				ind->means[i] = (unsigned int)(rndGen() % 2);// *nodes[i].hasDroneAcess;
			}	
			ind->means[numVariables - 1] = 0;
			ind->means[0] = 0;


			//Shuffle the sequence of the main path 
			std::shuffle(ind->solution.begin() + 1, ind->solution.end() - 1, rndGen);
			//ind->solution = { 0, 4, 8, 2, 9, 10, 3, 1, 7, 5, 6, 11 };
			ind->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
		}
		
	}
}

void HGenFS::processInd(Individual* ind)
{
	double truckTimeSum = 0;
	double waitTimeSum = 0;
	double totalPenaulty = 0;

	ind->penaulty = 0;

	
	int prec = 0;
	ind->syncLoss = 0;
	for (int i = 1; i < ind->solution.size(); i++)
	{

		if (ind->means[i] == 0) //Truck Delivery
		{
			truckTimeSum += (*timeTruck)[prec][ind->solution[i]];
			prec = ind->solution[i];
		}
		else
		{
			waitTimeSum += ind->getSyncDiff(i).first;
			
			if (nodes[ind->solution[i]].hasDroneAcess == 0)
			{
				ind->penaulty += 10;
			}

		}
	}
	//truckTimeSum += (*timeTruck)[prec][numVariables - 1];
	ind->truckTime = truckTimeSum;	
	ind->fit = truckTimeSum + waitTimeSum + ind->penaulty;
}




void HGenFS::processPopulation(std::string problemType)
{
	
	for (auto& ind : *newPop)
	{		
		processInd(ind);

		if (false)//ind->truckTime < bestTruck) 
		{
			bestTruck = ind->truckTime;	
			
			if (printResults)
			{												
				std::cout << "\nBestTruck: " << bestTruck;
			}		
		}
		
		if (localSearchNum > 0)
		{
			localSearch(ind, localSearchNum[0],localSearchNum[1], localSearchParams[0], localSearchParams[1]);
		}
	}

	std::vector<Individual*>* auxPointer = evaluatedPop;
	evaluatedPop = newPop; //new population become evaluated	
	newPop = auxPointer;

	auto sortRuleLambda = [](Individual* const& ind1, Individual* const& ind2) {return (ind1->fit < ind2->fit); };

	std::sort(evaluatedPop->begin(), evaluatedPop->end(), sortRuleLambda);
}

Individual* HGenFS::selection(std::string method, Individual* anotherParent, std::vector<int>* usedList)
{
	if (method == "tournament")
	{
		//std::shuffle(tournamentGroup.begin(), tournamentGroup.end(), rndGen);
		std::sort(tournamentGroup.begin(), tournamentGroup.begin() + tournamentSize);
		double rnd = (double)rndGen();		

		for (int i = 0; i < tournamentSize - 1; i++)
		{
			if (rnd / RAND_MAX < tournamentP)
			{
				if ((*evaluatedPop)[tournamentGroup[i]] != anotherParent)
					return (*evaluatedPop)[tournamentGroup[i]];
			}
		}
		return (*evaluatedPop)[tournamentGroup[tournamentSize - 1]]; //Last in the tournament is selected
	}

	if (method == "roullete") 
	{		
		double rnd = (double)rndGen();
		int randPos = (int)rnd % tournamentGroup.size();		
		
		for (int i = randPos; i < tournamentSize -1; i++)
		{			
			double rnd2 = (double)rndGen();
			if (rnd / RAND_MAX < (1 - (best - (*evaluatedPop)[i]->fit)/fitRange));
			{
				if ((*evaluatedPop)[i] != anotherParent)
					return (*evaluatedPop)[i];
			}
		}
		return (*evaluatedPop)[tournamentSize - 1]; //Last in the tournament is selected
	}
	
	if (method == "roullete2") 
	{				
								
		while(true)
		{
			double rnd = (double)rndGen();
			unsigned int randPos = (unsigned int)rnd % ((*usedList).size() - 1);
						
			if (rnd / RAND_MAX < best / (*evaluatedPop)[randPos]->fit) // best - (*evaluatedPop)[randPos]->fit) / fitRange));
			{
				if ((*evaluatedPop)[randPos] != anotherParent)
				{
					//(*usedList)[i] = 1;
					return (*evaluatedPop)[randPos];
				}
			}
		}
		//return (*evaluatedPop)[tournamentSize - 1]; //Last in the tournament is selected
	}
	return nullptr;
}

void HGenFS::localSearch(Individual* ind, int ns, int nd,  bool checkPath, bool checkEndurance )
{
		
	for (int i = 0; i < ns; i++)
	{
		double rnd = (double)rndGen();
		unsigned int randIdx = (unsigned int)rnd % (numVariables - 2) + 1; //1 -10

		double rnd2 = (double)rndGen();
		unsigned int randIdx2 = (unsigned int)rnd % (numVariables - 2) + 1; //1 -10			

		//Swap Search			
		std::vector<int> bakMeans = ind->means;
		std::vector<std::vector<int>> baKdronePath = ind->dronePath; //C

		double bakFit = ind->fit;
		int bak = ind->solution[randIdx2];

		ind->solution[randIdx2] = ind->solution[randIdx];
		ind->solution[randIdx] = bak;
		ind->CreateDronePath(checkPath, checkEndurance);
		processInd(ind);

		if (bakFit < ind->fit)
		{								
			for (int j = 0; j < nd; j++)
			{							
				double rnd = (double)rndGen();
				unsigned int randIdx = (unsigned int)rnd % (numVariables - 2) + 1; //1 -10
				
				//Swap Search			
				std::vector<int> bakMeans = ind->means;
				std::vector<std::vector<int>> baKdronePath = ind->dronePath; //C
				double bakFit = ind->fit;			
												
				if (ind->means[randIdx]  == 1) 
				{
					ind->means[randIdx] = 0;
					ind->dronePath[randIdx][0] = 0;
					ind->dronePath[randIdx][1] = 0;
				}
				else
				{
					ind->means[randIdx] = 1;
								
				}
												
				ind->CreateDronePath(true, true);
				processInd(ind);
				
				if (bakFit < ind->fit)
				{
					ind->means = bakMeans;
					ind->dronePath = baKdronePath;					
					processInd(ind);
				}
				else
				{
					//break;
				}				
			}
		}
			
		if (bakFit < ind->fit)
		{
			ind->means = bakMeans;
			ind->dronePath = baKdronePath;
			ind->solution[randIdx] = ind->solution[randIdx2];
			ind->solution[randIdx2] = bak;			
			processInd(ind);
		}
		else
		{
			//break;
		}
	
	}		
}


void HGenFS::childrenFromCrossOver(int child1, int child2, std::string type, std::string selectionType, std::vector<int>* usedList)
{	
	if (type == "swap")
	{
		Individual* parent1 = selection(selectionType,nullptr, usedList);
		Individual* parent2 = selection(selectionType, parent1,usedList);

		(*newPop)[child1]->solution = parent1->solution;
		(*newPop)[child1]->means = parent2->means;		
		(*newPop)[child1]->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
						
		(*newPop)[child2]->solution = parent2->solution;
		(*newPop)[child2]->means = parent1->means;
		(*newPop)[child2]->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
	}

	if (type == "mix1")
	{		
		Individual* parent1 = selection(selectionType, nullptr, usedList);
		Individual* parent2 = selection(selectionType, parent1, usedList);
		
		unsigned int cut1 = ((unsigned int)rndGen()% (parent1->solution.size() - 3)) + 1;
		unsigned int cut2 = (rndGen() % (parent1->solution.size() - cut1 - 1)) + cut1 + 1;
		//std::cout << "\n" << cut1 <<" | " << cut2 << " c1 " << child1 << " c2 " << child2;
		//Define if the crossover will be in truck ou drone path
		unsigned int isTruck = 0;// rndGen() % 2;

		std::set<int, greater<int>> included1;
		std::set<int, greater<int>> included2;

		//(*newPop)[child1]->printSol();
		//(*newPop)[child2]->printSol();		
		
		//Adding truck or drone path in the Children
		for (unsigned int i = cut1; i <= cut2; i++)
		{			
			if (parent1->means[i] == isTruck)// || nodes[i].hasDroneAcess == 0)
			{
				(*newPop)[child1]->solution[i] = parent1->solution[i];
				(*newPop)[child1]->means[i] = 0;
				included1.insert(parent1->solution[i]);
			}
			else
			{
				(*newPop)[child1]->solution[i] = -1;
				(*newPop)[child1]->means[i] = 1;
			}
			
			if (parent2->means[i] == isTruck)// || nodes[i].hasDroneAcess == 0)
			{
				(*newPop)[child2]->solution[i] = parent2->solution[i];
				(*newPop)[child2]->means[i] = 0;
				included2.insert(parent2->solution[i]);
			}
			else
			{
				(*newPop)[child2]->solution[i] = -1;
				(*newPop)[child2]->means[i] = 1;
			}
		}
		unsigned int p1counter = 0;
		unsigned int p2counter = 0;

		//std::cout << "\nMix2";
		//(*newPop)[child1]->printSol();
		//(*newPop)[child2]->printSol();

		//Completing paths using another parents sequence
		for (unsigned int i = 1; i < cut1; i++)
		{

			while (included1.find(parent2->solution[++p2counter]) != included1.end());
			(*newPop)[child1]->solution[i] = parent2->solution[p2counter];
			(*newPop)[child1]->means[i] = parent2->means[p2counter];// *nodes[i].hasDroneAcess;
			included1.insert(parent2->solution[p2counter]);

			while (included2.find(parent1->solution[++p1counter]) != included2.end());
			(*newPop)[child2]->solution[i] = parent1->solution[p1counter];
			(*newPop)[child2]->means[i] = parent1->means[p1counter];// *nodes[i].hasDroneAcess;
			included2.insert(parent1->solution[p1counter]);
		}
	

		for (unsigned int i = cut1; i <= cut2; i++)
		{
			if ((*newPop)[child1]->solution[i] == -1)//Not filled before
			{
				while (included1.find(parent2->solution[++p2counter]) != included1.end());
				(*newPop)[child1]->solution[i] = parent2->solution[p2counter];
				(*newPop)[child1]->means[i] = parent2->means[p2counter];// *nodes[i].hasDroneAcess;
				included1.insert(parent2->solution[p2counter]);
			}

			if ((*newPop)[child2]->solution[i] == -1)//Not filled before
			{

				while (included2.find(parent1->solution[++p1counter]) != included2.end());
				(*newPop)[child2]->solution[i] = parent1->solution[p1counter];
				(*newPop)[child2]->means[i] = parent1->means[p1counter];// *nodes[i].hasDroneAcess;
				included2.insert(parent1->solution[p1counter]);
			}
		}


		for (unsigned int i = cut2 + 1; i < parent1->solution.size() - 1; i++)
		{

			while (included1.find(parent2->solution[++p2counter]) != included1.end());
			(*newPop)[child1]->solution[i] = parent2->solution[p2counter];
			(*newPop)[child1]->means[i] = parent2->means[p2counter];// *nodes[i].hasDroneAcess;
			included1.insert(parent2->solution[p2counter]);


			while (included2.find(parent1->solution[++p1counter]) != included2.end());
			(*newPop)[child2]->solution[i] = parent1->solution[p1counter];
			(*newPop)[child2]->means[i] = parent1->means[p1counter];// *nodes[i].hasDroneAcess;
			included2.insert(parent1->solution[p1counter]);
		}

		(*newPop)[child1]->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
		(*newPop)[child2]->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
		
	}
}


void HGenFS::newGeneration()
{
	std::shuffle(tournamentGroup.begin(), tournamentGroup.end(), rndGen);
	fitRange = evaluatedPop->front()->fit - evaluatedPop->back()->fit;
	best = evaluatedPop->front()->fit;

	//std::vector<int> usedList;// (populationSize, 0);
	
	for (unsigned int i = 0; i < newPop->size(); i++)
	{
		//Hold elite (the 'n' Best)
		if (i < eliteSize)
		{
			Individual* indPtr = (*newPop)[i];
			(*newPop)[i] = (*evaluatedPop)[i];
			(*evaluatedPop)[i] = indPtr;

			//double rnd = (double)rand();
			//double rnd0_1 = (rnd / RAND_MAX); //0 - 1
			//int randIdx = (int)rnd % (numVariables - 2) + 1; //1 -10
			
			//Mutation
			double rnd = (double)rndGen(); 
			//std::cout << "\nRND: " << rnd;

			if ( (rnd / RAND_MAX) < mtRate && i > 3)
			{
				
				unsigned int randIdx = (unsigned int)rnd % (numVariables - 2) + 1; //1 -10

				//std::cout << "\nrnd:" << rnd << "\nID : " << randIdx << " i : " <<i;

				if (i % 2 == 0)
				{
					if ((*newPop)[i]->means[randIdx] == 0)// && nodes[randIdx].hasDroneAcess == 1)
					{
						(*newPop)[i]->means[randIdx] = 1 ;// !(*newPop)[i]->means[randIdx];
						(*newPop)[i]->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
					}
					else
					{
						(*newPop)[i]->means[randIdx] = 0;	
					}
				}
				else
				{
					//Swap Mutation
					if (randIdx != numVariables - 2)
					{
						int bak = (*newPop)[i]->solution[randIdx + 1];
						(*newPop)[i]->solution[randIdx + 1] = (*newPop)[i]->solution[randIdx];
						(*newPop)[i]->solution[randIdx] = bak;
					}
					else
					{
						int bak = (*newPop)[i]->solution[randIdx -1];
						(*newPop)[i]->solution[randIdx - 1] = (*newPop)[i]->solution[randIdx];
						(*newPop)[i]->solution[randIdx] = bak;

					}
				}
				(*newPop)[i]->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
			}


		}
		else if (i < csSize + eliteSize)
		{
			//Make CrossOvers
			childrenFromCrossOver(i, i+1, "swap", "roullete2", &tournamentGroup);// "tournament"); //roullete			
			i++;

			//Mutation
			double rnd = (double)rndGen();
			//std::cout << "\nRND: " << rnd;

			if ((rnd / RAND_MAX) < mtRate )
			{

				unsigned int randIdx = (unsigned int)rnd % (numVariables - 2) + 1; //1 -10

				//std::cout << "\nrnd:" << rnd << "\nID : " << randIdx << " i : " <<i;

				if (i % 2 == 0)
				{
					(*newPop)[i]->means[randIdx] = 1;// *nodes[randIdx].hasDroneAcess;// !(*newPop)[i]->means[randIdx];
					(*newPop)[i]->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
				}
				else
				{
					//Swap Mutation
					if (randIdx != numVariables - 2)
					{
						int bak = (*newPop)[i]->solution[randIdx + 1];
						(*newPop)[i]->solution[randIdx + 1] = (*newPop)[i]->solution[randIdx];
						(*newPop)[i]->solution[randIdx] = bak;
					}
					else
					{
						int bak = (*newPop)[i]->solution[randIdx - 1];
						(*newPop)[i]->solution[randIdx - 1] = (*newPop)[i]->solution[randIdx];
						(*newPop)[i]->solution[randIdx] = bak;

					}
				}
				(*newPop)[i]->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
			}
		}
		else
		{			
			
			std::shuffle((*newPop)[i]->solution.begin() + 1, (*newPop)[i]->solution.end() - 1, rndGen);
			for (int j = 1; j < numVariables - 1; j++)
			{
				(*newPop)[i]->means[j] = (unsigned int)(rndGen() % 2);// *nodes[j].hasDroneAcess;
			}
			//(*newPop)[i]->solution = std::vector<int>{ 0,5,7,2,6,1,8,9,4,3,10,11 };
			(*newPop)[i]->CreateDronePath(dronePathSyncSearch, pathCheckEndurance);
		}
	}
}

HGenFS::~HGenFS()
{
	for (auto& ind : (*evaluatedPop))
	{
		delete ind; 
	}
	for (auto& ind : (*newPop))
	{
		delete ind;
	}
}

void runHGenFS(HGenFS* testGen, int generations, int seed, std::string problemName)
{
	std::stringstream generationsResult;

	mtx.lock();
	std::cout << "\nSeed: " << seed << " -> Generations: " << generations;
	mtx.unlock();

	testGen->seed = seed;
	testGen->genFirstPopulation("random");

	//Proccess First Population
	testGen->processPopulation("TSPD");
	//testGen->processPopulation("TSPD");

	// Start measuring time
	auto begin = std::chrono::high_resolution_clock::now();	

	//Iterate over generations
	for (int g = 1; g <= generations; g++)
	{
		//for (int i = 0; i < indsPrint; i++)	(*testGen->evaluatedPop)[i]->printSol();

		//std::cout << "\n\nAntes:";
		//for (int i = 0; i < (*testGen->evaluatedPop)[i]->solution.size(); i++) (*testGen->evaluatedPop)[i]->printSol();

		testGen->newGeneration();
		testGen->processPopulation("TSPD");


		if (saveGenerations) generationsResult << testGen->evaluatedPop->front()->fit << ";";

		auto end = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

		mtx.lock();

		if (printResults)
		{
			//Print Population
			if (indsPrint > 0) std::cout << "\nGeneration: " << g;
			for (int i = 0; i < indsPrint; i++)	(*testGen->evaluatedPop)[i]->printSol();

		}

		if ((*testGen->evaluatedPop)[0]->fit < bestAll && (*testGen->evaluatedPop)[0]->penaulty == 0)
		{
			//std::cout << "\nSeed:" << seed;
			(*testGen->evaluatedPop)[0]->printSol();
			std::cout << " - in: " << g;
			bestAll = (*testGen->evaluatedPop)[0]->fit;
			testGen->bestIn = g;			
			std::string newBest = "\n" + std::to_string(elapsed.count() * 1e-9) + ";" + std::to_string(g) + ";" + (*testGen->evaluatedPop)[0]->getSolutionCsv();
			evolList.push_back(newBest);

		}
		mtx.unlock();

		if (elapsed.count() * 1e-9 > stopTime) break;
	}

	// Stop measuring time and calculate the elapsed time
	auto end = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

	mtx.lock();

	//for (int i = 0; i < indsPrint; i++)	(*testGen->evaluatedPop)[i]->printSol();

	std::cout << "\nSeed: " << seed << " -> " << testGen->evaluatedPop->front()->fit << "(" << testGen->evaluatedPop->front()->penaulty << ")" << " Time: " << elapsed.count() * 1e-9 << " sec";
	testGen->evaluatedPop->front()->printSol();



	if (saveGenerations)
	{
		ofstream myfile;
		myfile.open(resultsPath + problem + "_generations.csv", std::ios::app);
		myfile << "\n" << generationsResult.str();
		myfile.close();
	}

	if (saveMain)
	{
		ofstream myfile;
		myfile.open(resultsPath + problem + "_main.csv", std::ios::app);
		for (int i = 0; i < indsPrint + 1; i++)
		{
			myfile << "\n" << seed << ";" << i << ";" << testGen->bestIn << ";" << (*testGen->evaluatedPop)[i]->getSolutionCsv();
		}
		myfile.close();
	}

	if (saveEvolution)
	{
		ofstream myfile;
		myfile.open(resultsPath + problem + "_evolution.csv", std::ios::app);
		for (auto line : evolList) myfile << line;
		myfile.close();
	}
	
	mtx.unlock();	 
}


int main()
{
		
	std::vector<int> tests = {443 };
	std::vector<std::string> problems;
			
	printResults = true;
	saveGenerations = false;
	saveMain = true;
	saveEvolution = true;
	indsPrint = 0;	
	stopTime = 30;
	
	int generations = 300;
	int populationSize = 3000;
	int solutionSize = 11;
	int seeds = 4; // Will be the number of threads


	bool pathSearch = false;
	bool pathCheckEndurance = false;
	int localSearchN[2] = { 20, 1 };//TODO:Checar Local Search
	bool localSearchParams[2] = { false,false };
	
	
	std::cout << "\nRunning: ";
	/*for (auto test : tests)
	{
		for (int i = 1; i <= 12; i++)
		{
			problems.push_back(std::to_string(test) + "v" + std::to_string(i));
			std::cout << " | " << std::to_string(test) + "v" + std::to_string(i);
		}
	}*/
	
	//problems = { "437v6","437v12","440v6","440v7","440v8","440v9","443v7","443v10","443v11"};
	problems = { "uniform-alpha_1-51-n10", "uniform-alpha_1-52-n10", "uniform-alpha_1-53-n10", "uniform-alpha_1-54-n10", "uniform-alpha_1-55-n10", "uniform-alpha_1-56-n10", "uniform-alpha_1-57-n10", "uniform-alpha_1-58-n10", "uniform-alpha_1-59-n10", "uniform-alpha_1-60-n10" };

	

	for (auto p : problems)
	{
		std::cout << p << ",";
	}

	ofstream myfile;
	myfile.open(resultsPath + "group_result.csv", std::ios::app);

	myfile << "\n\nGenerations;" << generations;
	myfile << "\nPopulation;" << populationSize;
	myfile << "\nSeeds;" << seeds;
	myfile << "\nLocalSearchN;" << localSearchN[0] << ";" << localSearchN[1];
	myfile << "\nlocalSearchParams;" << localSearchParams[0] << ";" << localSearchParams[1];
	myfile << "\npathSearch;" << pathSearch;
	myfile << "\npathCheckEndurance;" << pathCheckEndurance;


	myfile << "\nproblem;time;bestIn;fit;syncLoss;penaulty;";
	for (int i = 0; i < solutionSize; i++)	myfile << "sol" << i << ";";
	myfile << "means;";
	for (int i = 0; i < solutionSize; i++)	myfile << "mean" << i << ";";
	myfile << "dronepath;";
	for (int i = 0; i < solutionSize; i++)	myfile << "dp" << i << ".0;" << "dp" << i << ".1;";
	myfile.close();

	for (auto pb : problems)
	{		
		problem = pb;
		
		std::cout << "\n\nStarting Problem: " << problem;
		
		if (saveGenerations)
		{
			std::ofstream myfile;
			myfile.open(resultsPath + problem + "_generations.csv");
			for (int g = 0; g < generations; g++) myfile << g << ";";
			myfile.close();
		}

		if (saveMain)
		{
			ofstream myfile;
			myfile.open(resultsPath + problem + "_main.csv");

			myfile << "seed;n;bestIn;fit;syncLoss;penaulty;";
			for (int i = 0; i < solutionSize; i++)	myfile << "sol" << i << ";";
			myfile << "means;";
			for (int i = 0; i < solutionSize; i++)	myfile << "mean" << i << ";";
			myfile << "dronepath;";
			for (int i = 0; i < solutionSize; i++)	myfile << "dp" << i << ".0;" << "dp" << i << ".1;";

			myfile.close();
		}

		if (saveEvolution)
		{
			ofstream myfile;
			myfile.open(resultsPath + problem + "_evolution.csv");
			myfile << "time;bestIn;fit;syncLoss;penaulty;";
			for (int i = 0; i < solutionSize; i++)	myfile << "sol" << i << ";";
			myfile << "means;";
			for (int i = 0; i < solutionSize; i++)	myfile << "mean" << i << ";";
			myfile << "dronepath;";
			for (int i = 0; i < solutionSize; i++)	myfile << "dp" << i << ".0;" << "dp" << i << ".1;";

			myfile.close();
		}
	
		std::vector<std::thread*> theads;
		for (int seed = 0; seed < seeds; seed++)
		{
			HGenFS* testGen = new HGenFS(populationSize, solutionSize, 0, 11, seed, problem, pathProblems);
			testGen->dronePathSyncSearch = pathSearch;
			testGen->localSearchNum[0] = localSearchN[0];
			testGen->localSearchNum[1] = localSearchN[1];
			testGen->localSearchParams[0] = localSearchParams[0];
			testGen->localSearchParams[1] = localSearchParams[1];
			testGen->pathCheckEndurance = pathCheckEndurance;

			theads.push_back(new std::thread(runHGenFS, testGen, generations, seed, problem));
		}

		for (auto& t : theads)
		{
			t->join();
		}

		if (evolList.size() > 0)
		{
			ofstream myfile;
			myfile.open(resultsPath + "group_result.csv", std::ios::app);
			myfile << "\n" << problem << ";" << evolList.back();
			myfile.close();
		}
		evolList.clear();
		bestAll = 9999999;
	}
	system("pause");
	return 0;
}
