#include <iostream>
#include <fstream>
#include <unistd.h>
#include <json.hpp>
#include <glob.hh>
#include <simulator.hh>

bool  g_showProgressBar;
float g_closeEnough;
float g_randomWalkwayRadius;
float g_attractionRadius;
uint32_t g_currTimeSim;

uint32_t g_timeExecMakeAgents;
uint32_t g_timeExecCal;
uint32_t g_timeExecSim;
uint32_t g_timeExecSimQuad;

std::vector<std::string> g_logZonesDensity;

omp_lock_t lock_agentsInQuad;

std::map<std::string, model_t> model_map = {
	{"ShortestPath",   ShortestPath},
	{"FollowTheCrowd", FollowTheCrowd},
	{"RandomWalkway",  RandomWalkway}
};

void loadDataFrom(std::string fileIn, json& dataOut){
	
	std::ifstream ifs;
	
	ifs.open(fileIn,std::ifstream::in);
	if(ifs.fail()) {
	    std::cerr << "Open error in file:"<<  fileIn << std::endl;
	    exit(EXIT_FAILURE);
	}

	ifs >> dataOut;
	ifs.close();
	
	if(dataOut.empty()){
	    std::cerr << "Can't load data from:"<<  fileIn << std::endl;
	    exit(EXIT_FAILURE);
	}
}


int main(int argc,char** argv) {
	char c;
	
	json settings;
	json area_zone;
	json initial_zones;
	json reference_zones;
	json reference_point;

	while((c=getopt(argc,argv,"s:"))!=-1) {
	        switch(c) {
	        case 's': {
	            std::ifstream ifs;
	            ifs.open(optarg,std::ifstream::in);
	            ifs >> settings;
	            ifs.close();
	            break;
	        }
		}
	}

	if(settings.empty()) {
	    std::cerr << "Mandatory parameter -s <config.json> needed" << std::endl;
	    exit(EXIT_FAILURE);
	}
	
	std::string map_osrm;
	std::string area_zone_file;
	std::string initial_zones_file;
	std::string reference_zones_file;
	std::string reference_point_file;
	
	try {
		map_osrm             = settings["input"]["map"].get<std::string>();
		area_zone_file       = settings["input"]["area"].get<std::string>();
		initial_zones_file   = settings["input"]["initial_zones"].get<std::string>();
		reference_zones_file = settings["input"]["reference_zones"].get<std::string>();
		reference_point_file = settings["input"]["reference_point"].get<std::string>();
	}catch (json::exception &e){
		std::cerr << "Error in get action from 'input' section in <config.json>:" << std::endl;
		std::cerr << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
		
	std::ifstream ifs;
	ifs.open(map_osrm,std::ifstream::in);
	if(ifs.fail()) {
	    std::cerr << "Error in file:"<<  map_osrm << std::endl;
	    exit(EXIT_FAILURE);
	}
	ifs.close();
	
	loadDataFrom(area_zone_file      , area_zone);
	loadDataFrom(initial_zones_file  , initial_zones);
	loadDataFrom(reference_zones_file, reference_zones);	
	loadDataFrom(reference_point_file, reference_point);
	
	//Reset counters
	g_timeExecMakeAgents = 0;
	g_timeExecCal        = 0;
	g_timeExecSim        = 0;
	g_timeExecSimQuad    = 0;

	omp_init_lock(&lock_agentsInQuad);
	
	Simulator sim(settings, initial_zones, reference_zones, reference_point, area_zone, map_osrm);

	sim.calibrate();
	sim.run();	
	sim.showTimeExec();
	
	omp_destroy_lock(&lock_agentsInQuad);  

	return(EXIT_SUCCESS);
}
