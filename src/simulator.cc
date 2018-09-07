#include <simulator.hh>


class Timer{
 private:
  typedef std::chrono::high_resolution_clock clock;
  typedef std::chrono::duration<double, std::ratio<1> > second;

  std::chrono::time_point<clock> start_time; ///< Last time the timer was started
  double accumulated_time;                   ///< Accumulated running time since creation
  bool running;                              ///< True when the timer is running

 public:
  Timer(){
    accumulated_time = 0;
    running          = false;
  }

  ///Start the timer. Throws an exception if timer was already running.
  void start(){
    if(running)
      throw std::runtime_error("Timer was already started!");
    running=true;
    start_time = clock::now();
  }

  ///Stop the timer. Throws an exception if timer was already stopped.
  ///Calling this adds to the timer's accumulated time.
  ///@return The accumulated time in seconds.
  double stop(){
    if(!running)
      throw std::runtime_error("Timer was already stopped!");

    accumulated_time += lap();
    running           = false;

    return accumulated_time;
  }

  ///Returns the timer's accumulated time. Throws an exception if the timer is
  ///running.
  double accumulated(){
    if(running)
      throw std::runtime_error("Timer is still running!");
    return accumulated_time;
  }

  ///Returns the time between when the timer was started and the current
  ///moment. Throws an exception if the timer is not running.
  double lap(){
    if(!running)
      throw std::runtime_error("Timer was not started!");
    return std::chrono::duration_cast<second> (clock::now() - start_time).count();
  }

  ///Stops the timer and resets its accumulated time. No exceptions are thrown
  ///ever.
  void reset(){
    accumulated_time = 0;
    running          = false;
  }
};


///@brief Manages a console-based progress bar to keep the user entertained.
///
///Defining the global `NOPROGRESS` will
///disable all progress operations, potentially speeding up a program. The look
///of the progress bar is shown in ProgressBar.hpp.
class ProgressBar{
 private:
  uint32_t total_work;    ///< Total work to be accomplished
  uint32_t next_update;   ///< Next point to update the visible progress bar
  uint32_t call_diff;     ///< Interval between updates in work units
  uint32_t work_done;
  uint16_t old_percent;   ///< Old percentage value (aka: should we update the progress bar) TODO: Maybe that we do not need this
  Timer    timer;         ///< Used for generating ETA

  ///Clear current line on console so a new progress bar can be written
  void clearConsoleLine() const {
    std::cerr<<"\r\033[2K"<<std::flush;
  }

 public:
  ///@brief Start/reset the progress bar.
  ///@param total_work  The amount of work to be completed, usually specified in cells.
  void start(uint32_t total_work){
    timer = Timer();
    timer.start();
    this->total_work = total_work;
    next_update      = 0;
    call_diff        = total_work/200;
    old_percent      = 0;
    work_done        = 0;
    clearConsoleLine();
  }

  ///@brief Update the visible progress bar, but only if enough work has been done.
  ///
  ///Define the global `NOPROGRESS` flag to prevent this from having an
  ///effect. Doing so may speed up the program's execution.
  void update(uint32_t work_done0){
    //Provide simple way of optimizing out progress updates
    #ifdef NOPROGRESS
      return;
    #endif

    //Quick return if this isn't the main thread
    if(omp_get_thread_num()!=0)
      return;

    //Update the amount of work done
    work_done = work_done0;

    //Quick return if insufficient progress has occurred
    if(work_done<next_update)
      return;

    //Update the next time at which we'll do the expensive update stuff
    next_update += call_diff;

    //Use a uint16_t because using a uint8_t will cause the result to print as a
    //character instead of a number
    uint16_t percent = (uint8_t)(work_done*omp_get_num_threads()*100/total_work);

    //Handle overflows
    if(percent>100)
      percent=100;

    //In the case that there has been no update (which should never be the case,
    //actually), skip the expensive screen print
    if(percent==old_percent)
      return;

    //Update old_percent accordingly
    old_percent=percent;

    //Print an update string which looks like this:
    //  [================================================  ] (96% - 1.0s - 4 threads)
    std::cerr<<"\r\033[2K["
             <<std::string(percent/2, '=')<<std::string(50-percent/2, ' ')
             <<"] ("
             <<percent<<"% - "
             <<std::fixed<<std::setprecision(1)<<timer.lap()/percent*(100-percent)
			 << "s" <<std::flush;
            // <<"s - "
            // <<omp_get_num_threads()<< " threads)"<<std::flush;
  }

  ///Increment by one the work done and update the progress bar
  ProgressBar& operator++(){
    //Quick return if this isn't the main thread
    if(omp_get_thread_num()!=0)
      return *this;

    work_done++;
    update(work_done);
    return *this;
  }

  ///Stop the progress bar. Throws an exception if it wasn't started.
  ///@return The number of seconds the progress bar was running.
  double stop(){
    clearConsoleLine();

    timer.stop();
    return timer.accumulated();
  }

  ///@return Return the time the progress bar ran for.
  double time_it_took(){
    return timer.accumulated();
  }

  uint32_t cellsProcessed() const {
    return work_done;
  }
};


Simulator::Simulator(void) {

}
Simulator::Simulator(const json &_fsettings,const json &_finitial_zones,const json &_freference_zones,const json &_freference_point,const std::string &_map_osrm) {
    static thread_local std::random_device device;
    static thread_local std::mt19937 rng(device());

    this->_fsettings = _fsettings;
    this->_projector = LocalCartesian(_freference_point["features"][0]["geometry"]["coordinates"][1],_freference_point["features"][0]["geometry"]["coordinates"][0],0,Geocentric::WGS84());
    this->_router    = Router(_freference_point,_map_osrm);
	

	for(auto& feature : _freference_zones["features"]){
		this->_reference_zones.push_back(Zone(_freference_point, feature));
	}

	for(auto& feature : _finitial_zones["features"]){
		this->_initial_zones.push_back(Zone(_freference_point, feature));
	}

	uint32_t id = 0;

	std::uniform_int_distribution<uint32_t> zone(0, this->_initial_zones.size()-1);

	for(auto& fagent : _fsettings["agents"]) {
		for(uint32_t i = 0; i<uint32_t(fagent["number"]); i++,id++) {
			Point2D position = this->_initial_zones[zone(rng)].generate();

			auto agent = Agent(id,position,fagent["speed"]["min"],fagent["speed"]["max"],model_t(this->_hash(fagent["model"].get<std::string>())));
			this->_agents.push_back(agent);
		}
	}
	
	//Se crea el ambiente con los agentes recien creados. 
	this->_env = Environment(this->_agents);
}
void Simulator::calibrate(void) {
	
	uint32_t calibration_time = this->_fsettings["calibration"].get<uint32_t>();
	
	std::cout << "Ajustando posición inicial de los agentes..." << std::endl;

	ProgressBar pg;
	pg.start(calibration_time-1);
	for(uint32_t t = 0; t < calibration_time; t++) {
		pg.update(t);
		for(auto& agent : this->_agents){
			if(this->_routes[agent.id()].empty()){
				auto response = this->_router.route(agent.position(),RANDOMWALKWAY_RADIUS);
				this->_routes[agent.id()] = response.path();
			}
			agent.random_walkway(this->_routes[agent.id()]);
		}
	}

	std::cout << std::endl;
	std::cout << "Ajustando reglas de los agentes... " << std::endl;
	
	pg.start(this->_agents.size()-1);

#pragma omp parallel for firstprivate(_router) shared(_agents) //schedule(dynamic,8)
	for(uint32_t i = 0; i < this->_agents.size(); i++){
		pg.update(i);
		
		Agent agent = _agents[i];
		
		switch(agent.model()) {
			case SHORTESTPATH: {
				double distance = DBL_MAX;
				for(auto &reference_zone : this->_reference_zones) {
				    auto response = this->_router.route(agent.position(),reference_zone.generate());
				    if(response.distance() < distance) {
				        distance = response.distance();
				        this->_routes[agent.id()] = response.path();
				    }
				}
				break;
			}
			case RANDOMWALKWAY:  {					            
				break;
			}
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
	this->run( this->_fsettings["duration"] );
}
void Simulator::run(const uint32_t &_duration) {
	std::cout << std::endl << "Simulando..." << std::endl;
	
	bool save_to_disk = this->_fsettings["output"]["filesim-out"].get<bool>();
	uint32_t interval = this->_fsettings["output"]["interval"].get<uint32_t>();

    //Router router=this->_router; //Esta declaración no corresponde aquí <03/09/2018>
    //Environment _env(this->_agents); //_env es un atributo del objeto, no una variable local. Mover al constructor. <03/09/2018>
	
	ProgressBar pg;
    pg.start(_duration-1);
	
    for(uint32_t t = 0; t < _duration; t++) {
        //std::cout << "time: "<< t << std::endl;
		//printProgress(double(t)/double(_duration - 1) );
		pg.update(t);
		
        if(save_to_disk && ((t%interval) == 0)) {
			this->save(t); //GAM: <16/08/2018>, WAS this->save(t/SAVE) 
		} 

#pragma omp parallel for firstprivate(_router) shared(_env) //schedule(dynamic,8) 
        for(uint32_t i = 0; i < this->_agents.size(); i++){
            switch(this->_agents[i].model()) {
            case SHORTESTPATH: {
                this->_agents[i].follow_path(this->_routes[this->_agents[i].id()]);
                break;
            }
            case RANDOMWALKWAY: {
                if(this->_routes[this->_agents[i].id()].empty()){  
					//std::cout << "t=" << t<<  ", Agent[" <<  i << "] con ruta vacía"<<std::endl;
					auto response = _router.route(this->_agents[i].position(),RANDOMWALKWAY_RADIUS);
					this->_routes[this->_agents[i].id()] = response.path();
                }
                this->_agents[i].random_walkway(this->_routes[this->_agents[i].id()]);
                break;
            }
            case FOLLOWTHECROWD: {
                Agent::Neighbors neighbors = _env.neighbors_of(this->_agents[i],ATTRACTION_RADIUS,SHORTESTPATH);
                
                if(neighbors.empty()){
                  if(this->_routes[this->_agents[i].id()].empty()){    
                     auto response = _router.route(this->_agents[i].position(),RANDOMWALKWAY_RADIUS);
                     this->_routes[this->_agents[i].id()] = response.path();
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
        _env.update(this->_agents);
    }
	
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
	
}
