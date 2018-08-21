#include <simulator.hh>


void printProgress (double percentage)
{
	std::string PBSTR = "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
	int PBWIDTH = PBSTR.length();
	
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf ("\r%3d%% [%.*s%*s]", val, lpad, PBSTR.c_str(), rpad, "");
    fflush (stdout);
}


Simulator::Simulator(void) {

}
Simulator::Simulator(const json &_fsettings,const json &_finitial_zones,const json &_freference_zones,const json &_freference_point,const std::string &_map_osrm) {
    static thread_local std::random_device device;
    static thread_local std::mt19937 rng(device());

    this->_fsettings=_fsettings;
    this->_projector=LocalCartesian(_freference_point["features"][0]["geometry"]["coordinates"][1],_freference_point["features"][0]["geometry"]["coordinates"][0],0,Geocentric::WGS84());
    this->_router=Router(_freference_point,_map_osrm);

    for(auto& feature : _freference_zones["features"])
        this->_reference_zones.push_back(Zone(_freference_point,feature));

    for(auto& feature : _finitial_zones["features"])
        this->_initial_zones.push_back(Zone(_freference_point,feature));

    uint32_t id=0U;
    std::uniform_int_distribution<uint32_t> zone(0U,this->_initial_zones.size()-1U);
    for(auto& fagent : _fsettings["agents"]) {
        for(uint32_t i=0U; i<uint32_t(fagent["number"]); i++,id++) {
            Point2D position=this->_initial_zones[zone(rng)].generate();
            auto agent=Agent(id,position,fagent["speed"]["min"],fagent["speed"]["max"],model_t(this->_hash(fagent["model"].get<std::string>())));
            this->_agents.push_back(agent);
        }
    }
}
void Simulator::calibrate(void) {
    std::cout << "calibrating" << std::endl;
	
	uint32_t calibration_time = this->_fsettings["calibration"].get<uint32_t>();

    for(uint32_t t=0; t<calibration_time; t++) {
		//uint32_t it = 0;
        for(auto& agent : this->_agents){
            if(this->_routes[agent.id()].empty()){
               auto response=this->_router.route(agent.position(),RANDOMWALKWAY_RADIUS);
               this->_routes[agent.id()]=response.path();
            }
            agent.random_walkway(this->_routes[agent.id()]);
			//printProgress(double(it++)/double( this->_agents.size() ));
        }
    }

	uint32_t it = 0;
    for(auto& agent : this->_agents) {
        switch(agent.model()) {
        case SHORTESTPATH: {
            double distance=DBL_MAX;
            for(auto &reference_zone : this->_reference_zones) {
                auto response=this->_router.route(agent.position(),reference_zone.generate());
                if(response.distance()<distance) {
                    distance=response.distance();
                    this->_routes[agent.id()]=response.path();
                }
            }
            break;
        }
        case RANDOMWALKWAY: break;
        case FOLLOWTHECROWD: break;
        case WORKINGDAY: break;
        default: {
            std::cerr << "error::simulator_constructor::unknown_mobility_model::\"" << agent.model() << "\"" << std::endl;
            exit(EXIT_FAILURE);
        }
        }
		printProgress(double(it++)/double(this->_agents.size()));
    }
}
double distance(Agent a,Agent b){
   return(sqrt(CGAL::squared_distance(a.position(),b.position())));
}
void Simulator::run(void) {
    this->run( this->_fsettings["duration"] );
}
void Simulator::run(const uint32_t &_duration) {
    std::cout << "simulating" << std::endl;
	
	bool save_to_disk = this->_fsettings["output"]["filesim-out"].get<bool>();
	uint32_t interval = this->_fsettings["output"]["interval"].get<uint32_t>();

    Router router=this->_router;
    Environment env(this->_agents);
    
    for(uint32_t t=0U; t<_duration; t++) {
        std::cout << "time: "<< t << std::endl;
        if(save_to_disk && ((t%interval)==0)) {
			this->save(t); //GAM: 16/08/2018, WAS this->save(t/SAVE) 
		} 

        //#pragma omp parallel for firstprivate(router) schedule(dynamic,8) shared(env)
        for(uint32_t i=0U;i<this->_agents.size();i++){
            switch(this->_agents[i].model()) {
            case SHORTESTPATH: {
                this->_agents[i].follow_path(this->_routes[this->_agents[i].id()]);
                break;
            }
            case RANDOMWALKWAY: {
                if(this->_routes[this->_agents[i].id()].empty()){    
                     auto response=_router.route(this->_agents[i].position(),RANDOMWALKWAY_RADIUS);
                     this->_routes[this->_agents[i].id()]=response.path();
                }
                this->_agents[i].random_walkway(this->_routes[this->_agents[i].id()]);
                break;
            }
            case FOLLOWTHECROWD: {
                Agent::Neighbors neighbors=env.neighbors_of(this->_agents[i],ATTRACTION_RADIUS,SHORTESTPATH);
                
                if(neighbors.empty()){
                  if(this->_routes[this->_agents[i].id()].empty()){    
                     auto response=_router.route(this->_agents[i].position(),RANDOMWALKWAY_RADIUS);
                     this->_routes[this->_agents[i].id()]=response.path();
                   }
                  this->_agents[i].random_walkway(this->_routes[this->_agents[i].id()]);
                }
                else
                   this->_agents[i].follow_the_crowd(neighbors);
                break;
            }
              case WORKINGDAY: break;
            }
        }
        env.update(this->_agents);
    }
	
	//getchar();
	
}

Simulator::~Simulator(void) {
    this->_agents.clear();
    this->_initial_zones.clear();
    this->_reference_zones.clear();
}
void Simulator::save(const uint32_t &_t) {
    static std::map<model_t,int> types={{SHORTESTPATH,0},{FOLLOWTHECROWD,1},{RANDOMWALKWAY,2}};//model


	std::string nameFile = this->_fsettings["output"]["filesim-prefix"].get<std::string>()+std::to_string(_t)+this->_fsettings["output"]["filesim-sufix"].get<std::string>();
    std::string pathFile = this->_fsettings["output"]["filesim-path"].get<std::string>() + "/" + nameFile ;
	std::ofstream ofs(pathFile);
		
    for(auto& agent : this->_agents) {
        double latitude,longitude,h;
        this->_projector.Reverse(agent.position()[0],agent.position()[1],0,latitude,longitude,h);
        ofs << agent.id() << " " << latitude << " " << longitude << " " << types[agent.model()] <<std::endl;
    }
	
	/*auto image_out = _fsettings["output"]["image-out"].get<bool>();
	
	if(image_out){
		std::string nameFilePng = this->_fsettings["output"]["image-prefix"].get<std::string>()+std::to_string(_t)+ ".png";
		std::string pathFilePng = this->_fsettings["output"]["image-path"].get<std::string>() + "/" + nameFilePng ;
		
		std::string image_helper = this->_fsettings["output"]["image-helper"].get<std::string>();
	
		std::string cmd = "cat " + pathFile + " | " + image_helper + " > " + pathFilePng;
		//std::cout << cmd << std::endl;
		system(cmd.c_str());	
	}*/
	
	
	
	//showfile( pathFilePng.c_str() );
}
