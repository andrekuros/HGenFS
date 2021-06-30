#include "QGen.h"

using namespace std;

QGen::QGen(int populationSize, double mutationRate, double csRate, int numVariables, int varMin, int varMax, int seed, std::string problemPath) :
	populationSize(populationSize), mtRate(mutationRate), csRate(csRate), numVariables(numVariables), 
	varMin(varMin), varMax(varMax), seed(seed), problemPath(problemPath)
{
	best = 10;
	elite = 2;
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

void QGen::genFirstPopulation(std::string option)
{
	init = option;	
	rndGen.seed(seed);	

	if (init == "random")
	{
		for (auto& ind : *newPop)
		{
			//Set a randon number for how much clients will be
			//served by the truck and store in the path
			//int numTruckDevs = rndGen() % (numVariables-2);			
			//for (int n = 1; n <= numTruckDevs; n++)	ind.truckPath[n] = n;			
			for (int i = 0; i < numVariables; i++)
			{
				ind->solution[i] = i;
				ind->means[i] =  (rndGen() % 2)* nodes[i].hasDroneAcess;
			}			
			//Shuffle the sequence of the main path 
			std::shuffle(ind->solution.begin()+1, ind->solution.end()-1, rndGen);
						
			ind->CreateDronePath();
		}
	}
					
	//for (auto &item : ind.solution)
	//	std::cout << item << "-";
}

void QGen::processPopulation(std::string problemType)
{
	
	double totalTime = 0;
	for (auto& ind : *newPop)
	{		
		double truckTimeSum = 0;
		double waitTimeSum = 0;		

		if (problemType == "TSPD")
		{
			int prec = 0;
			for (int i = 1; i < ind->solution.size() ; i ++)
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
			truckTimeSum += (*timeTruck)[prec][numVariables-1];
			
		}
		else
		{
			std::cout << "\nUnknown Problem: " << problemType;
			exit(1);
		}				
		ind->fit = truckTimeSum + waitTimeSum;
	}
	
	void* auxPointer = evaluatedPop;		
	evaluatedPop = newPop; //new pupulation become evaluated	
	newPop = evaluatedPop;

	auto sortRuleLambda = [](Individual* const& ind1, Individual* const& ind2)
	{
		return (ind1->fit < ind2->fit);
	};

	std::sort(newPop->begin(), newPop->end(), sortRuleLambda);
}

void QGen::newGeneration()
{
	int i = 0;
	for (auto& ind : *newPop)
	{
		if (i >= 3)
		{
			std::shuffle(ind->solution.begin()+1, ind->solution.end()-1, rndGen);
			ind->CreateDronePath();
		}
		i++;
	}
}

QGen::~QGen()
{
	
}

std::string runQGen(QGen* testGen, int generations, int seed, std::string filename)
{
	std::stringstream result;

	//for (int g = 0; g < generations; g++) result << g << ";";
	
	testGen->seed = seed;
	testGen->genFirstPopulation("random");
	mtx.lock();
	std::cout << "\nSeed: " << seed << " -> Generation: " << generations;	
	mtx.unlock();

	//Iterate over generations
	for (int g = 1; g < generations; g++)
	{		
		testGen->processPopulation("TSPD");
		testGen->newGeneration();
		//std::cout << "\nGen: " << g;

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

			/*
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
			std::cout << "]  FIT: " << ind->fit;
			*/
			if (++i > 1) break;
		}		
	}

	mtx.lock();
	std::cout << "\nSeed: " << seed << " -> " << testGen->evaluatedPop->front()->fit;
	ofstream myfile;
	myfile.open(filename,std::ios::app);
	myfile << "\n" << result.str();
	myfile.close();
	mtx.unlock();
	return result.str();
}



int main()
{		
	int generations = 300;

	std::string path = "problems/FSTSP_10_customer_problems/20140810T123";
	std::string problem = "443v12";
	
	std::ofstream myfile;
	myfile.open("results/" + problem + ".csv");
	for (int g = 0; g < generations; g++) myfile << g << ";";
	myfile.close();
	
	std::vector<std::thread*> theads;

	for (int seed = 0; seed < 10; seed++)
	{		
		QGen* testGen = new QGen(20000, 0.01, 0.2, 12, 0, 11, seed, path + problem);			
		theads.push_back(new std::thread(runQGen, testGen, generations, seed,  "results/" + problem + ".csv" ));
		//auto future = std::async();
		//std::string simple = future.get();		
	}

	for (auto& t : theads)
	{
		t->join();
	}
	
	return 0;
}
