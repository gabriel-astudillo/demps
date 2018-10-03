#include <simulator.hh>
#include <environment.hh>

std::shared_ptr<Environment> Simulator::_env;


Simulator::Simulator(void) {

}

Simulator::Simulator(const json &_fsettings,const json &_finitial_zones,const json &_freference_zones,const json &_freference_point,const json &_fmap_zone,const std::string &_map_osrm) {
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());

	std::vector<Agent> _agents;

	timeExecCal = 0;
	timeExecSim = 0;

	this->_fsettings = _fsettings;
	
	_duration        = this->_fsettings["duration"];
	_calibrationTime = this->_fsettings["calibration"].get<uint32_t>();
	_saveToDisk      = this->_fsettings["output"]["filesim-out"].get<bool>();
	_interval        = this->_fsettings["output"]["interval"].get<uint32_t>();
	_filesimPrefix   = this->_fsettings["output"]["filesim-prefix"].get<std::string>();
	_filesimSufix    = this->_fsettings["output"]["filesim-sufix"].get<std::string>();
	_filesimPath     = this->_fsettings["output"]["filesim-path"].get<std::string>();
	
	// showProgressBar debe ser global al proyecto
	bool showProgressBar = this->_fsettings["output"]["progressBar"].get<bool>();
	

	//Tamaño del cuadrante
	uint32_t quadSize = this->_fsettings["quadSize"].get<uint32_t>(); //quadSize[m] x quadSize[m]

	//Se crea el ambiente vacío. 
	_env = std::make_shared<Environment>();

	_env->setReferencePoint(_freference_point);
	_env->setRouter(_map_osrm);
	_env->setProjector();
	_env->setReferenceZones(_freference_zones);
	_env->setInitialZones(_finitial_zones);
	
	_env->setGrid(_fmap_zone, quadSize);
	
	/// Se crea un agente temporal para
	// inicializar la variable estática _myEnv
	// de la clase Agent
	auto fooAgent = Agent();
	fooAgent.setEnvironment(_env);

	std::uniform_int_distribution<uint32_t> zone(0, _env->getInitialZones().size()-1);

	std::cout << "Creando agentes..." << std::endl;	
	uint32_t id = 0;
	ProgressBar pg;
	for(auto& fagent : _fsettings["agents"]) {
		uint32_t totalAgents = uint32_t(fagent["number"]);
		pg.start(totalAgents);
		 
		for(uint32_t i = 0; i < totalAgents; i++, id++) {
			if(showProgressBar){
				pg.update(i);
			}
			
			Point2D position = _env->getInitialZone(zone(rng)).generate();

			auto agent = Agent(id,\
				position,\
				fagent["speed"]["min"],\
				fagent["speed"]["max"],\
				model_t(this->_hash(fagent["model"].get<std::string>()))\
				); 

			_agents.push_back(agent);
					
		}
	}
	
	if(showProgressBar){
		std::cout << std::endl;
	}
	
	//Se agregan al ambiente con los agentes recien creados. 
	_env->addAgents(_agents);
}

void Simulator::calibrate(void) {
	
	bool showProgressBar      = this->_fsettings["output"]["progressBar"].get<bool>();
	
	//
	// Se realiza un ajuste en la posición inicial de los agentes,
	// para que queden en las calles del mapa. Esto se realiza
	// en dos pasos:
	// 1) Se ajusta el modelo de movilidad de todos los agentes a RANDOMWALK
	// 2) Los agentes caminan un tiempo determinado para que finalmente
	//    queden en las calles
	//
	
	//
	// AJUSTE: PASO 1
	//
	std::cout << "Ajustando posición inicial de los agentes...(1/2)" << std::endl;
	auto start = std::chrono::system_clock::now(); //Measure Time
	
	ProgressBar pg;
	pg.start(_env->getTotalAgents()-1);
	
	#pragma omp parallel for 
	for(uint32_t i = 0; i < _env->getTotalAgents(); i++){
		if(showProgressBar){
			pg.update(i);
		}
		
		Agent* agent = _env->getAgent(i);
	
		auto response = _env->getRouter()->route(agent->position(),RANDOMWALKWAY_RADIUS);
		_env->_routes[agent->id()] = response.path();					
	}

	if(showProgressBar){
		std::cout << std::endl;
	}
	
	//
	// AJUSTE: PASO 2
	//
	std::cout << "Ajustando posición inicial de los agentes...(2/2)" << std::endl;
	pg.start(_calibrationTime-1);
	for(uint32_t t = 0; t < _calibrationTime; t++) {
		if(showProgressBar){
			pg.update(t);
		}	

		#pragma omp parallel for 
		for(uint32_t i = 0; i < _env->getTotalAgents(); i++){
			Agent* agent = _env->getAgent(i);
			
			if(_env->_routes[agent->id()].empty()){
				auto response = _env->getRouter()->route(agent->position(),RANDOMWALKWAY_RADIUS);
				_env->_routes[agent->id()] = response.path();				
			}
			agent->random_walkway(_env->_routes[agent->id()]);				
		}
	}

	if(showProgressBar){
		std::cout << std::endl;
	}
	
	//
	// Se ajustan las reglas de los agentes
	//
	std::cout << "Ajustando reglas de los agentes... " <<  std::endl;
	_env->adjustAgentsRules();
	

	auto end = std::chrono::system_clock::now(); //Measure Time
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	timeExecCal += elapsed.count();
	
	if(showProgressBar){
		std::cout << std::endl;
	}
}

double distance(Agent a,Agent b){
	return(sqrt(CGAL::squared_distance(a.position(),b.position())));
}


void Simulator::run() {
	std::cout << "Simulando..." << std::endl;
	
	bool showProgressBar = this->_fsettings["output"]["progressBar"].get<bool>();
	
	ProgressBar pg;
	pg.start(_duration-1);
	
	for(uint32_t t = 0; t < _duration; t++) {
        
		if(showProgressBar){
			pg.update(t);
		}
		
		if(_saveToDisk && ((t%_interval) == 0)) {
			this->save(t); //GAM: <16/08/2018>, WAS this->save(t/SAVE) 
		} 

		auto start = std::chrono::system_clock::now(); //Measure Time

		_env->updateAgents();
		
		auto end = std::chrono::system_clock::now(); //Measure Time
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		timeExecSim += elapsed.count();
	}
	
	if(showProgressBar){
		std::cout << std::endl;
	}	
	
}

Simulator::~Simulator(void) {
	;
}
void Simulator::save(const uint32_t &_t) {
	static std::map<model_t,int> types={{SHORTESTPATH,0},{FOLLOWTHECROWD,1},{RANDOMWALKWAY,2}};//model
		  
	std::string nameFile = _filesimPrefix + std::to_string(_t) + _filesimSufix ;
	std::string pathFile = _filesimPath + "/" + nameFile ;
	
	
	std::ofstream ofs(pathFile);
		
	for( auto& agent : _env->getAgents() ) {
		double latitude,longitude,h;
		_env->getProjector().Reverse(agent.position()[0],agent.position()[1],0,latitude,longitude,h); 
		ofs << agent.id() << " " << latitude << " " << longitude << " " << types[agent.model()] <<std::endl;
	}
	
}

void Simulator::showTimeExec(void){
	
	std::cout << _fsettings["duration"] << ":" 
		<< _fsettings["agents"][0]["number"] << ":" 
		<< _fsettings["agents"][1]["number"] << ":" 
		<< _fsettings["agents"][2]["number"] << ":" 
		<< timeExecCal << ":" 
		<< timeExecSim 
		<< std::endl;
}


