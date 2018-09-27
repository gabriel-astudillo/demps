#include <iostream>
#include <fstream>
#include <unistd.h>
#include <json.hpp>
#include <glob.hh>
#include <simulator.hh>

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
	
	map_osrm                  = settings["input"]["map"].get<std::string>();
	try {
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

	ifs.open(area_zone_file,std::ifstream::in);
	if(ifs.fail()) {
	    std::cerr << "Error in file:"<<  area_zone_file << std::endl;
	    exit(EXIT_FAILURE);
	}

	ifs >> area_zone;
	ifs.close();

	ifs.open(initial_zones_file,std::ifstream::in);
	if(ifs.fail()) {
	    std::cerr << "Error in file:"<<  initial_zones_file << std::endl;
	    exit(EXIT_FAILURE);
	}

	ifs >> initial_zones;
	ifs.close();

	ifs.open(reference_zones_file,std::ifstream::in);
	if(ifs.fail()) {
	    std::cerr << "Error in file:"<<  reference_zones_file << std::endl;
	    exit(EXIT_FAILURE);
	}
	ifs >> reference_zones;
	ifs.close();

	ifs.open(reference_point_file,std::ifstream::in);
	if(ifs.fail()) {
	    std::cerr << "Error in file:"<<  reference_point_file << std::endl;
	    exit(EXIT_FAILURE);
	}
	ifs >> reference_point;
	ifs.close();


	if(map_osrm.empty() || area_zone.empty() || initial_zones.empty() || reference_zones.empty() || reference_point.empty()) {
	    std::cerr << "Check file path in input section" << std::endl;
	    exit(EXIT_FAILURE);
	} 
	

	Simulator sim(settings,initial_zones,reference_zones,reference_point,area_zone,map_osrm);

	sim.calibrate();
	sim.run();
	
	sim.showTimeExec();

	return(EXIT_SUCCESS);
}
