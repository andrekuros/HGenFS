#include "Individual.h"


Individual::Individual(std::vector<std::vector<double>>* timeTruck, std::vector<std::vector<double>>* timeDrone, double droneEndurance, int size, double seed) :
	timeDrone(timeDrone), timeTruck(timeTruck), droneEndurance(droneEndurance) , size(size)
{
	rndGen.seed(seed);	
	solution.resize(size);
	means.resize(size);
	dronePath.resize(size);
	for (auto& pair : dronePath) pair.resize(2); //2 for a while Dep->Recover idx in the solution
}

double Individual::getSyncDiff(int solIdx)
{
	//If is not a drone delivery constumer we do not 
	//Calculate de sincronization difference
	if (means[solIdx] == 0) return 0;
	
	double truckTravelTime = 0;
	int prec = dronePath[solIdx][0];
	//Follow the truck path from the dep and recover of the drone
	for (int i = dronePath[solIdx][0] + 1; i <= dronePath[solIdx][1]; i++)
	{
		if (means[i] == 0)
		{
			truckTravelTime += (*timeTruck)[prec][i];
			prec = i;
		}
	}
	//Time to the Drone fly from A to Cotumer to C plus 2 minutes (launch and retrieve);
	double droneTravelTime = (*timeDrone)[solution[dronePath[solIdx][0]]][solution[solIdx]] +
							 (*timeDrone)[solution[solIdx]][solution[dronePath[solIdx][1]]] + 1; //1 minute SL
	
	if (droneTravelTime + 1 > droneEndurance) return 100000000; //Temporary Penalty to Discard Infeaseble

	if (droneTravelTime > truckTravelTime || solIdx == solution.size()-1)
	{
		return abs(droneTravelTime - truckTravelTime) + 1 ; //1 minute SR
	}
	else 
	{
		return 1; //Noeffect in the time, because the truck do not need to wait
	}

	
}


bool Individual::CreateDronePath()
{	

	for (int i = 1; i < solution.size()-1; i++)
	{
		int lastLand = -1;

		if (means[i] == 1) //Drone delivery is 1 -> solution[i] is the costumer
		{						

			//Search for possible departures positions 		
			std::vector<int> depAlternatives;
			std::vector<int> recoverAlternatives;

			double truckTravel = (*timeTruck)[solution[i - 1]][solution[i + 1]];

			if (truckTravel > droneEndurance)
			{
				means[i] = 0;
				return false;
			}
			else
			{				
				depAlternatives.emplace_back(i - 1);
				recoverAlternatives.emplace_back(i + 1);
			}

			int lastVisited = solution[i-1];

			//Look for otions to departure before
			for (int n = i - 2; n >= 0; n--)
			{
				if (means[n] == 0) //Visited by the Truck
				{					
					truckTravel += (*timeTruck)[solution[n]][lastVisited];
					lastVisited = solution[n];

					if (truckTravel <= droneEndurance) depAlternatives.emplace_back(n);					
					else break; //Costumers before thar are unfeaseble					
				}
			}	
			
			if (depAlternatives.size() == 0)
			{		
				means[i] = 0; //deliver with the truck	
				return false;
			}
			std::shuffle(depAlternatives.begin(), depAlternatives.end(), rndGen);

			//Loof for possible recover positions 						
			truckTravel = (*timeTruck)[solution[i - 1]][solution[i + 1]]; //Minimun Time
			lastVisited = solution[i-1];
			
			//Start from next truck stop up to max drone endurance 
			for (int n = i + 2; n < solution.size(); n++)
			{
				if (means[n] == 0) //Visited by the Truck
				{
					truckTravel += (*timeTruck)[lastVisited][n < solution[n]];
					lastVisited = solution[n];

					if (truckTravel <= droneEndurance) recoverAlternatives.emplace_back(n);
					else break; //Next costumers are unreachable 																	
				}
			}

			dronePath[i][0] = depAlternatives[rndGen() % depAlternatives.size()];
			dronePath[i][1] = recoverAlternatives[rndGen() % recoverAlternatives.size()]; 				
			
		}
	}
	return true; //Any dep point available to reach the costumer)
}