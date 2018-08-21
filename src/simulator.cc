#include <simulator.hh>

Simulator::Simulator(void) {

}
Simulator::Simulator(const json &_fsettings,const json &_finitial_zones,const json &_freference_zones,const json &_freference_point,const std::string &_map_osrm) {
    static thread_local std::random_device device;
    static thread_local std::mt19937 rng(device());

    this->_fsettings=_fsettings;
    this->_projector=LocalCartesian(_freference_point["features"][0]["geometry"]["coordinates"][1],_freference_point["features"][0]["geometry"]["coordinates"][0],0,Geocentric::WGS84());
    
	//std::cout << _freference_point << std::endl;
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

    for(uint32_t t=0U; t<CALIBRATION_TIME; t++) {
        for(auto& agent : this->_agents){
            if(this->_routes[agent.id()].empty()){
               auto response=this->_router.route(agent.position(),RANDOMWALKWAY_RADIUS);
               this->_routes[agent.id()]=response.path();
            }
            agent.random_walkway(this->_routes[agent.id()]);
        }
    }

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
    }
}
double distance(Agent a,Agent b){
   return(sqrt(CGAL::squared_distance(a.position(),b.position())));
}
void Simulator::run(void) {
    this->run(this->_fsettings["duration"],true);
}
void Simulator::run(const uint32_t &_duration,const bool &_save_to_disk) {
    std::cout << "simulating" << std::endl;

    Router router=this->_router;
    Environment env(this->_agents);
    
    for(uint32_t t=0U; t<_duration; t++) {
        std::cout << "time: "<< t << std::endl;
        if(_save_to_disk && ((t%SAVE)==0)) this->save(t/SAVE);

        #pragma omp parallel for firstprivate(router) schedule(dynamic,8) shared(env)
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
}

Simulator::~Simulator(void) {
    this->_agents.clear();
    this->_initial_zones.clear();
    this->_reference_zones.clear();
}
void Simulator::save(const uint32_t &_t) {
    static std::map<uint32_t,int> types={{9366416273040049814U,0},{10676684734677566718U,1},{5792789823329120861U,2}};//model

    std::ofstream ofs(this->_fsettings["output"]["directory-path"].get<std::string>()+"/"+this->_fsettings["output"]["filename-prefix"].get<std::string>()+std::to_string(_t)+this->_fsettings["output"]["filename-sufix"].get<std::string>());

    for(auto& agent : this->_agents) {
        double latitude,longitude,h;
        this->_projector.Reverse(agent.position()[0],agent.position()[1],0,latitude,longitude,h);
        ofs << agent.id() << " " << latitude << " " << longitude << " " << types[agent.model()] <<std::endl;
    }
}
