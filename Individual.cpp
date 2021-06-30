#include "Individual.h"


Individual::Individual(std::vector<std::vector<double>>* timeTruck, std::vector<std::vector<double>>* timeDrone, double droneEndurance, int dronesAvailable, int size, double seed) :
	timeDrone(timeDrone), timeTruck(timeTruck), droneEndurance(droneEndurance), dronesAvailable(dronesAvailable) , size(size)
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
	double droneTravelTime = (*timeDrone)[ solution[ dronePath[solIdx][0] ]][ solution[solIdx] ] +
							 (*timeDrone)[ solution[solIdx] ][ solution[dronePath[solIdx][1] ] ] + 1; //1 minute SL
	
	if (droneTravelTime + 1 > droneEndurance || truckTravelTime + 1 > droneEndurance) return 100000000; //Temporary Penalty to Discard Infeaseble

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
	int lastLand = -1;

	for (int i = 1; i < solution.size()-1; i++)
	{			
		if (means[i] == 1) //Drone delivery is 1 -> solution[i] is the costumer
		{						

			//Check if there are drones available in the truck (only one now)
			if (i <= lastLand) //= means that can land and dep in same constumer 
			{
				means[i] = 0;
				continue;
			}
			
			//Search for possible departures and land positions 		
			std::vector<std::pair<int,int>> droneRouteAlternative;

			int landLimit = 10000000; 
			//Last land ensure that the drone is in the truck
			for (int n_dep = i - 1; n_dep >= 0 && n_dep > lastLand && landLimit > i; n_dep--)
			{				
				if (means[n_dep] == 0) //Visited by the Truck
				{
					double truckTravel = 0;// (*timeTruck)[n_back][n_fwd] < solution[n]];					
					int lastVisited = n_dep;

					for (int n_land = i + 1; n_land < solution.size(); n_land++)
					{
						if (means[n_land] == 0) //Visited by the Truck
						{
							truckTravel += (*timeTruck)[lastVisited][n_land];
							lastVisited = n_land;
							
							if (truckTravel <= droneEndurance) 
							{
								droneRouteAlternative.emplace_back(std::make_pair(n_dep, n_land));
							}
							else 
						    {
								if (droneRouteAlternative.size() > 0)
								{
									landLimit = droneRouteAlternative.back().second;
								}
								else
								{
									means[i] = 0;
									continue;
								}
								break; //Next costumers are unreachable 																	
							}
						}
					}
				}
			}
				
			if (droneRouteAlternative.size() > 0)
			{
				std::pair<int, int> route = droneRouteAlternative[rndGen() % droneRouteAlternative.size()];
				dronePath[i][0] = route.first;
				dronePath[i][1] = route.second;
				lastLand = dronePath[i][1];
			}
			else
			{
				means[i] = 0;
				continue;
			}
		}
	}

	return true; //Any dep point available to reach the costumer)
}