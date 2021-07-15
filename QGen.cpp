#include "QGen.h"

using namespace std;

QGen::QGen(int populationSize, int numVariables, int varMin, int varMax, int seed, std::string problemPath) :
	populationSize(populationSize), numVariables(numVariables), 
	varMin(varMin), varMax(varMax), seed(seed), problemPath(problemPath)
{
		
	best = 10;
	eliteProportion = 0.05;
	eliteSize =  (int)(populationSize * eliteProportion);
	
	csProportion = 0.5;
	csSize = (int)(populationSize * csProportion);
	csSearchRate = 50;

	mtRate = 0.02;

	truckProportion = 0.7;
	droneEndurance = 20;	
	dronesAvailable = 1;
	nodes.resize(numVariables);
	
	newPop = new std::vector<Individual*>;
	evaluatedPop = new std::vector<Individual*>;
	newPop->resize(populationSize);
	evaluatedPop->resize(populationSize);	

	timeTruck = new std::vector<std::vector<double>>;
	timeDrone = new std::vector<std::vector<double>>;
		
	for (auto& ind : *newPop)
		ind = new Individual(timeTruck, timeDrone, droneEndurance, dronesAvailable, numVariables,  rndGen());			

	for (auto& ind : *evaluatedPop)	
		ind = new Individual(timeTruck, timeDrone, droneEndurance, dronesAvailable, numVariables, rndGen());

	for (int i = 0; i < populationSize; i++)
	{
		tournamentGroup.push_back(i);
	}
	tournamentSize = (int)(populationSize * tournamentProportion);


	//LOAD Costs Truck	
	std::ifstream dataTruck(problemPath + "/tau.csv");
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
		std::cout << "Wrong File TrackCost: " << problemPath + "/tau.csv";				
	}

	//LOAD Costs Drone	
	std::ifstream dataDrone(problemPath + "/tauprime.csv");
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
	std::ifstream dataNodes(problemPath + "/nodes.csv");
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
				nodes[n].hasDroneAcess = false;
			}
			else
			{
				nodes[n].hasDroneAcess = !row[3];
			}						
			nodes[n].x = row[1];
			nodes[n].y = row[2];			
		}
		nodes.back().hasDroneAcess = false;
	}	
};

std::pair<double, double> QGen::fitFromSolutionTable(std::string cod)
{	
	auto search = solutionTable.find(cod);	
	return search == solutionTable.end() ? std::make_pair(-1.0,0.0): search->second;
}

void QGen::genFirstPopulation(std::string option)
{
	init = option;	
	rndGen.seed(seed);	

	if (init == "random")
	{
		for (auto& ind : *newPop)
		{			
			for (int i = 0; i < numVariables; i++)
			{
				ind->solution[i] =   i; //soltest[i];//
				ind->means[i] = (rndGen() % 2)* nodes[i].hasDroneAcess;
			}					
						
			//Shuffle the sequence of the main path 
			std::shuffle(ind->solution.begin()+1, ind->solution.end()-1, rndGen);									
			ind->CreateDronePath();
			ind->genCode();
		}

		for (auto& ind : *evaluatedPop)
		{
			for (int i = 0; i < numVariables; i++)
			{
				ind->solution[i] = i; //soltest[i];//
				ind->means[i] = (rndGen() % 2) * nodes[i].hasDroneAcess;
			}
			
			//Shuffle the sequence of the main path 
			std::shuffle(ind->solution.begin() + 1, ind->solution.end() - 1, rndGen);
			ind->CreateDronePath();
			ind->genCode();
		}
	}
}


void QGen::processPopulation(std::string problemType)
{
	
	double totalTime = 0;
	for (auto& ind : *newPop)
	{		
		if (ind->knownBefore.first == -1) //Not known before
		{
			double truckTimeSum = 0;
			double waitTimeSum = 0;
			double totalPenaulty = 0;

			ind->penaulty = 0;

			if (problemType == "TSPD")
			{
				int prec = 0;
				for (int i = 1; i < ind->solution.size(); i++)
				{
					if (ind->means[i] == 0) //Truck Delivery
					{
						truckTimeSum += (*timeTruck)[prec][ind->solution[i]];
						prec = ind->solution[i];
					}
					else
					{
						waitTimeSum += ind->getSyncDiff(i);
					}
				}
				truckTimeSum += (*timeTruck)[prec][numVariables - 1];

			}
			else
			{
				std::cout << "\nUnknown Problem: " << problemType;
				exit(1);
			}

			ind->fit = truckTimeSum + waitTimeSum + ind->penaulty;
			ind->genCode();
			ind->knownBefore = std::pair<double, double>(ind->fit, ind->penaulty);
			solutionTable[ind->code] = ind->knownBefore;
		}
		else 
		{
			ind->fit = ind->knownBefore.first;
			ind->penaulty = ind->knownBefore.second;
		}		
	}
	
	std::vector<Individual*>* auxPointer = evaluatedPop;
	evaluatedPop = newPop; //new population become evaluated	
	newPop = auxPointer;
	
	auto sortRuleLambda = [](Individual* const& ind1, Individual* const& ind2){return (ind1->fit < ind2->fit);};

	std::sort(evaluatedPop->begin(), evaluatedPop->end(), sortRuleLambda);
}

Individual* QGen::selection(std::string method, Individual* anotherParent)
{	
	if (method == "tournament")
	{				
		std::shuffle(tournamentGroup.begin(), tournamentGroup.end(), rndGen);				
		std::sort(tournamentGroup.begin(), tournamentGroup.begin() + tournamentSize);

		for (int i = 0; i < tournamentSize - 1; i++)
		{									
			if (rndGen() / RAND_MAX > tournamentP)
			{
				if ((*evaluatedPop)[i] != anotherParent)									
					return (*evaluatedPop)[i];				
			}	
		}
		return (*evaluatedPop)[tournamentSize - 1]; //Last in the tournament is selected
	}
	
}

void QGen::childrenFromCrossOver(int child1, int child2, std::string type)
{
	return;
	if (type == "swap")
	{								
		
		Individual* parent1 = selection("tournament");
		Individual* parent2 = selection("tournament", parent1);
			
		(*newPop)[child1]->solution = parent1->solution;
		(*newPop)[child1]->means = parent2->means;
		(*newPop)[child1]->CreateDronePath();
		(*newPop)[child1]->genCode();
		(*newPop)[child1]->knownBefore = fitFromSolutionTable((*newPop)[child1]->code);		

		if ((*newPop)[child1]->knownBefore.first != -1)
		{	
			for (int i = 0; i < csSearchRate; i++)
			{
				int idxSwap = rndGen() % ((*newPop)[child1]->solution.size()-3) + 1;
				int auxCostumer = (*newPop)[child1]->solution[idxSwap];
				(*newPop)[child1]->solution[idxSwap] = (*newPop)[child1]->solution[idxSwap+1];
				(*newPop)[child1]->solution[idxSwap + 1] = auxCostumer;
				(*newPop)[child1]->CreateDronePath();
				(*newPop)[child2]->genCode();
				(*newPop)[child1]->knownBefore = fitFromSolutionTable((*newPop)[child1]->code);
					
				if ((*newPop)[child1]->knownBefore.first == -1)
				{
					break;
				}
				else 
				{
					for (int j = 0; j < numVariables; j++)
					{
						(*newPop)[child1]->means[j] = (rndGen() % 2) * nodes[j].hasDroneAcess;
					}
					(*newPop)[child1]->CreateDronePath();
					(*newPop)[child2]->genCode();
					(*newPop)[child1]->knownBefore = fitFromSolutionTable((*newPop)[child1]->code);

				}
				if ((*newPop)[child1]->knownBefore.first == -1)
				{
					break;
				}

			}
		}
		
		(*newPop)[child2]->solution = parent2->solution;
		(*newPop)[child2]->means = parent1->means;
		(*newPop)[child2]->CreateDronePath();
		(*newPop)[child2]->genCode();
		(*newPop)[child2]->knownBefore = fitFromSolutionTable((*newPop)[child2]->code);			

		if ((*newPop)[child2]->knownBefore.first != -1)
		{
			for (int i = 0; i < csSearchRate; i++)
			{
				int idxSwap = rndGen() % ((*newPop)[child2]->solution.size() - 3) + 1;
				int auxCostumer = (*newPop)[child2]->solution[idxSwap];
				(*newPop)[child2]->solution[idxSwap] = (*newPop)[child2]->solution[idxSwap + 1];
				(*newPop)[child2]->solution[idxSwap + 1] = auxCostumer;
				(*newPop)[child2]->CreateDronePath();
				(*newPop)[child2]->genCode();
				(*newPop)[child2]->knownBefore = fitFromSolutionTable((*newPop)[child2]->code);

				if ((*newPop)[child2]->knownBefore.first == -1)
				{
					break;
				}
				else
				{
					for (int j = 0; j < numVariables; j++)
					{
						(*newPop)[child2]->means[j] = (rndGen() % 2) * nodes[j].hasDroneAcess;
					}
					(*newPop)[child2]->CreateDronePath();
					(*newPop)[child2]->genCode();
					(*newPop)[child2]->knownBefore = fitFromSolutionTable((*newPop)[child2]->code);
				}
				if ((*newPop)[child2]->knownBefore.first == -1)
				{
					break;
				}

			}
		}
		
	}
}


void QGen::newGeneration()
{		
				
	for (int i = 0; i < newPop->size(); i++)
	{
		//Hold elite (the 'n' Best)
		if (i < eliteSize)
		{			
			(*newPop)[i]->solution = (*evaluatedPop)[i]->solution;
			(*newPop)[i]->means = (*evaluatedPop)[i]->means;
			(*newPop)[i]->dronePath = (*evaluatedPop)[i]->dronePath;			
			(*newPop)[i]->fit = (*evaluatedPop)[i]->fit;
			(*newPop)[i]->penaulty = (*evaluatedPop)[i]->penaulty;
			(*newPop)[i]->code = (*evaluatedPop)[i]->code;
			(*newPop)[i]->knownBefore = (*evaluatedPop)[i]->knownBefore;
		}
		else if (i < csSize + eliteSize)
		{
			childrenFromCrossOver(i, ++i, "swap");
		}
		else
		{			
			//(*newPop)[i]->means = std::vector<int>{ 0,0,0,1,0,0,0,1,0,0,0,0 };

			std::shuffle((*newPop)[i]->solution.begin() + 1, (*newPop)[i]->solution.end() - 1, rndGen);
			for (int j = 0; j < numVariables; j++)
			{
				(*newPop)[i]->means[j] = (rndGen() % 2) * nodes[j].hasDroneAcess;
			}
			
			//(*newPop)[i]->solution = std::vector<int>{ 0,5,7,2,6,1,8,9,4,3,10,11 };

			(*newPop)[i]->CreateDronePath();
			(*newPop)[i]->genCode();
			(*newPop)[i]->knownBefore = fitFromSolutionTable((*newPop)[i]->code);
			(*newPop)[i]->fit = (*newPop)[i]->knownBefore.first;
			(*newPop)[i]->penaulty = (*newPop)[i]->knownBefore.second;

		}			
	}
}

QGen::~QGen()
{
	
}

std::string runQGen(QGen* testGen, int generations, int seed, std::string filename)
{
	std::stringstream result;

	bool printResults = false;

	//for (int g = 0; g < generations; g++) result << g << ";";
	
	testGen->seed = seed;
	testGen->genFirstPopulation("random");
	mtx.lock();
	std::cout << "\nSeed: " << seed << " -> Generation: " << generations;	
	mtx.unlock();

	testGen->processPopulation("TSPD");
	
	//Iterate over generations
	for (int g = 1; g <= generations; g++)
	{		
		
		testGen->newGeneration();

		testGen->processPopulation("TSPD");

		int varKnown = testGen->solutionTable.size() - testGen->lastKnown;
		testGen->lastKnown = testGen->solutionTable.size();
		
		if (printResults)
			std::cout << "\nGen: " << g << "Known: " << testGen->solutionTable.size() << "(+" << varKnown << ")";

		int i = 0;
		//Print Population
		for (auto& ind : *testGen->evaluatedPop)
		{
			if (i == 0)
			{
				//myfile << g;
				//for (auto& item : ind->solution) 
				//	myfile << item << ";";
				result << ind->fit << ";";				
			}

			if (printResults)
			{
				std::cout << "\nSol: [";
				for (auto& item : ind->solution) std::cout << item << " ";
				std::cout << "] - Means [";

				for (int i = 0; i < ind->means.size(); i++)
				{
					if (ind->means[i] == 0)
						std::cout << ind->means[i] << " ";
					else
						std::cout << ind->solution[ind->dronePath[i][0]] << "|" << ind->solution[ind->dronePath[i][1]] << " ";
				}
				std::cout << "]  FIT: " << ind->fit << "(+" << ind->penaulty << ")";
			}			
			if (++i > 30) break;
		}				
	}

	mtx.lock();
	std::cout << "\nSeed: " << seed << " -> " << testGen->evaluatedPop->front()->fit;

	//std::cout << "]  FIT: " << testGen->evaluatedPop->front()->fit << "(+" << testGen->evaluatedPop->front()->penaulty << ")";
	//std::cout << "\n" << testGen->evaluatedPop->front()->code;
	ofstream myfile;
	myfile.open(filename,std::ios::app);
	myfile << "\n" << result.str();
	myfile.close();
	mtx.unlock();
	return result.str();
}


int main()
{		
	std::vector<int> tests = { 443, 440, 443 };
	
	int generations = 300;

	std::string path = "problems/FSTSP_10_customer_problems/20140810T123";
	std::string problem = "437v1";
	
	std::ofstream myfile;
	myfile.open("results/" + problem + ".csv");
	for (int g = 0; g < generations; g++) myfile << g << ";";
	myfile.close();
	
	std::vector<std::thread*> theads;

	for (int seed = 0; seed < 10; seed++)
	{		
		QGen* testGen = new QGen(30000, 12, 0, 11, seed, path + problem);			
		theads.push_back(new std::thread(runQGen, testGen, generations, seed,  "results/" + problem + ".csv" ));
		//auto future = std::async();
		//std::string simple = future.get();		
	}

	for (auto& t : theads)
	{
		t->join();
	}
	getchar();
	return 0;
}
