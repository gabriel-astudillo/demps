#include <iostream>
#include <fstream>
#include <unistd.h>
#include <json.hpp>
#include <glob.hh>
#include <simulator.hh>

std::shared_ptr<LocalCartesian> projector;

int main(int argc,char** argv) {
    char c;
    std::string map_osrm;
    json area;
    json settings;
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
        std::cerr << "Mandatory parameter -s <settings.json> needed" << std::endl;
        exit(EXIT_FAILURE);
    }
	
	
	map_osrm                  = settings["input"]["map"].get<std::string>();
	
	auto initial_zones_file   = settings["input"]["initial_zones"].get<std::string>();
	auto reference_zones_file = settings["input"]["reference_zones"].get<std::string>();
	auto reference_point_file = settings["input"]["reference_point"].get<std::string>();
	
	std::ifstream ifs;
	
    ifs.open(map_osrm,std::ifstream::in);
	if(ifs.fail()) {
        std::cerr << "Error in file:"<<  map_osrm << std::endl;
        exit(EXIT_FAILURE);
	}
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
	
	
    if(map_osrm.empty() || initial_zones.empty() || reference_zones.empty() || reference_point.empty()) {
        std::cerr << "Check file path in input section" << std::endl;
        exit(EXIT_FAILURE);
    } 
	
	
	
    Simulator sim(settings,initial_zones,reference_zones,reference_point,map_osrm);

    sim.calibrate();
    sim.run();
	

    return(0);
}
