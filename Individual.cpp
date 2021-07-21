#include "individual.h"


Individual::Individual(std::vector<std::vector<double>>* timeTruck, std::vector<std::vector<double>>* timeDrone, double droneEndurance, int dronesAvailable, int size, double seed) :
	timeDrone(timeDrone), timeTruck(timeTruck), droneEndurance(droneEndurance), dronesAvailable(dronesAvailable), size(size)
{
	rndGen.seed(seed);
	solution.resize(size);
	means.resize(size);
	//hasDep.resize(size);
	dronePath.resize(size);
	for (auto& pair : dronePath) 
	{
		pair.resize(2);		//2 for a while Dep->Recover idx in the solution
		pair[0] = 0;
		pair[1] = 0;
	}
}

//void Individual::genCode()
//{
//	code = std::make_pair(solution, dronePath);
	
//}

std::pair<double,double>  Individual::getSyncDiff(int solIdx)
{

	//If is not a drone delivery constumer we do not 
	//Calculate de sincronization difference
	if (means[solIdx] == 0) return std::make_pair(0.0,0.0);

	double truckTravelTime = 0.0;
	int prec = dronePath[solIdx][0];

	//Follow the truck path from the dep and recover of the drone
	for (int i = dronePath[solIdx][0] + 1; i <= dronePath[solIdx][1]; i++)
	{
		if (means[i] == 0)
		{
			truckTravelTime += (*timeTruck)[solution[prec]][solution[i]];
			prec = i;
		}
	}

	//Time to the Drone fly from A to Cotumer to C plus 1 minutes (launch and retrieve);
	double droneTravelTime = (*timeDrone)[solution[dronePath[solIdx][0]]][solution[solIdx]] +
		(*timeDrone)[solution[solIdx]][solution[dronePath[solIdx][1]]]; //1 minute SL (*Sl do mnot contribute to endurance computation)

	double syncLostTime = 0.0;
	double syncInLast = 0.0;

	//Check Sync
	if (droneTravelTime > truckTravelTime || dronePath[solIdx][1] == solution.size() - 1)
	{
		if (droneTravelTime > truckTravelTime)
		{
			syncLostTime = fabs(droneTravelTime - truckTravelTime);
		}
		else
		{		
			//droneTravelTime = droneTravelTime + syncLostTime;
			syncLostTime = truckTravelTime - droneTravelTime + 1;
			syncLostTime = syncLostTime < 0 ? syncLostTime : 0;
			 
		}

	}
	else
	{
		if (dronePath[solIdx][0] != 0 ) //if departure from depot, drone delay the dep to dot not wait
			droneTravelTime = truckTravelTime; //Drone need to wait the truck		
	}

	//(dronePath[solIdx][1] != (solution.size() - 1))? 1 : 0
	//Add Penaulties
	double thisPenaulty = 0;
	if (((droneTravelTime + 1 > droneEndurance) && (dronePath[solIdx][1] != (solution.size() - 1))) 
		|| ((droneTravelTime + 1 > droneEndurance) && (dronePath[solIdx][1] == (solution.size() - 1))))
	{		
		thisPenaulty =  (droneTravelTime + 1 - droneEndurance) * penaultyFactor;
	}

	if ((truckTravelTime + 1 > droneEndurance) && dronePath[solIdx][1] != (solution.size() - 1)) //hasDep from HGA condition????
	{
		thisPenaulty += (truckTravelTime + 1 - droneEndurance) * penaultyFactor;
	}
	penaulty += thisPenaulty;
	syncLoss += syncLostTime;

	double adicionalTime = 1.0; //RecoverTime

	if (dronePath[solIdx][0] != 0)
	{
		adicionalTime += 1.0; //only SR (SL does not count) 
	}

	/*if (dronePath[solIdx][1] != (solution.size() - 1))
	{
		adicionalTime += 1.0; // add SR
	}*/
			
	std::pair<double, double> result{ syncLostTime + adicionalTime, thisPenaulty };
	return result; //1'SL + 1' SR (no SL from depot)
}


bool Individual::CreateDronePath(bool dronePathLocalSearch, bool checkDroneEndurance, bool checkSync)
{
	int lastLand = -1;
	
	//Reset dep control to be able to control Sr + Sl in the asame node (truckRestriction)
	//for (int i = 0; i < hasDep.size(); i++) hasDep[i] = false;
	droneFromDepot = false;
		
	for (int i = 1; i < solution.size() - 1; i++)
	{
		if (means[i] == 1 ) //Drone delivery is 1 -> solution[i] is the costumer
		{
			

			//Check if there are drones available in the truck (only one for now)
			if (i < lastLand) // only < means that can land and dep in same constumer 
			{
				means[i] = 0;
				dronePath[i][0] = 0;
				dronePath[i][1] = 0;
				continue;
			}

			//Search for possible departures and land positions 		
			std::vector<std::pair<int, int>> droneRouteAlternative;

			int landLimit = 10000000;

			bool lastInBack = false;

			//Last land ensure that the drone is in the truck
			for (int n_dep = i - 1; n_dep >= 0 && n_dep >= lastLand && landLimit > i; n_dep--)
			{
				if (means[n_dep] == 0) //Visited by the Truck
				{
					double truckTravel = 0;// (*timeTruck)[n_back][n_fwd] < solution[n]];					
					int lastVisited = n_dep;

					for (int n_land = i + 1; n_land < solution.size(); n_land++)
					{
						if (means[n_land] == 0) //Visited by the Truck
						{
							truckTravel += (*timeTruck)[solution[lastVisited]][solution[n_land]];
							lastVisited = n_land;

							if (truckTravel <= droneEndurance || !checkDroneEndurance)
							{
								if (rndGen() % 2 == 1)
								{
									droneRouteAlternative.emplace_back(std::make_pair(n_dep, n_land));
									lastInBack = true;
								}
								else
								{
									droneRouteAlternative.emplace(droneRouteAlternative.begin(), std::make_pair(n_dep, n_land));
									lastInBack = false;
								}																
							}
							else
							{
								if (droneRouteAlternative.size() > 0)
								{
									landLimit = lastInBack? droneRouteAlternative.back().second : droneRouteAlternative.front().second;
								}
								else
								{
									landLimit = -1; //Next costumers are unreachable
									break;
								}																								
							}
						}
					}
				}
			}

			if (droneRouteAlternative.size() > 0)
			{				
				int idxBestdiff = 0;
				double bestDiff = 999999;
				
				if (dronePathLocalSearch)
				{
					for (int k = 0; k < droneRouteAlternative.size(); k++)
					{
						//std::pair<int, int> route = droneRouteAlternative[0];
						//Muito estranho isso não era pra precisar
						//if (means[route.first] == 0 && means[route.second] == 0)
						//{
						std::pair<double, double> calcDiff = getSyncDiff(i);
						double diff = calcDiff.first + calcDiff.second;

						if (diff < bestDiff) {
							bestDiff = diff;
							idxBestdiff = k;
						}

						if (diff == 0)
						{

							dronePath[i][0] = droneRouteAlternative[k].first;
							dronePath[i][1] = droneRouteAlternative[k].second;

							//hasDep[droneRouteAlternative[k].first] = true;

							if (droneRouteAlternative[k].first == 0) droneFromDepot = true;

							lastLand = dronePath[i][1];
							break;
						}
						else
						{
							dronePath[i][0] = droneRouteAlternative[idxBestdiff].first;
							dronePath[i][1] = droneRouteAlternative[idxBestdiff].second;

							//hasDep[droneRouteAlternative[idxBestdiff].first] = true;

							if (droneRouteAlternative[idxBestdiff].first == 0) droneFromDepot = true;

							lastLand = dronePath[i][1];
						}
						//break;
					//}
					}
				}
				else 
				{
					dronePath[i][0] = droneRouteAlternative[0].first;
					dronePath[i][1] = droneRouteAlternative[0].second;

					//hasDep[droneRouteAlternative[0].first] = true;

					if (droneRouteAlternative[0].first == 0) droneFromDepot = true;

					lastLand = dronePath[i][1];

				}

				/*if (rndGen() % 2 == 1)
				{
					std::pair<int, int> route = droneRouteAlternative[0];
					means[route.second] = 0;				
					dronePath[i][0] = route.first;
					dronePath[i][1] = route.second;
					lastLand = dronePath[i][1];
				}*/
			}
			else
			{
				means[i] = 0;
				dronePath[i][0] = 0;
				dronePath[i][1] = 0;
				continue;
			}
		}
		else
		{
			dronePath[i][0] = 0;
			dronePath[i][1] = 0;
		}
	}
	return true; //Any dep point available to reach the costumer)
}


void Individual::printSol()
{

	std::cout << "\nSol: [";
	for (auto& item : solution) std::cout << item << " ";
	std::cout << "] -Means [";

	for (int i = 0; i < means.size(); i++)
	{
		if (means[i] == 0)
			std::cout << means[i] << " ";
		else
			std::cout << solution[dronePath[i][0]] << "|" << solution[dronePath[i][1]] << " ";
	}
	std::cout << "]  FIT: " << fit << "(+" << penaulty << ") -Truck: " << truckTime << "|sync: " << syncLoss;
}

std::string Individual::getSolutionCsv()
{

	std::stringstream solutionCsv;

	solutionCsv << fit << ";" << syncLoss << ";" << penaulty << ";";
	
	for (auto item : solution) solutionCsv << item << ";";

	solutionCsv << "means;";
	
	for (auto mean : means) solutionCsv << mean << ";";

	solutionCsv << "dronePath;";
	
	for (auto dp : dronePath) solutionCsv << dp[0] << ";" << dp[1] <<";";

	return solutionCsv.str();
						
}