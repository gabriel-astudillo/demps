#include <simulator.hh>
#include <environment.hh>

uint32_t getMaxMemory()
{
	struct rusage r_usage;
	getrusage(RUSAGE_SELF,&r_usage);
	//https://www.gnu.org/software/libc/manual/html_node/Resource-Usage.html
	uint32_t maxMemory = r_usage.ru_maxrss;//KB

	return(maxMemory);
}

Simulator::Simulator(void)
{

}

Simulator::Simulator(const json &_fsettings,const json &_finitial_zones,const json &_freference_zones, const json &_fmap_zone,const std::string &_map_osrm)
{
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());

	std::vector<Agent> _agents;

	this->_fsettings = _fsettings;

	_duration        = this->_fsettings["duration"];
	_calibrationTime = this->_fsettings["calibration"].get<uint32_t>();
	_saveToDisk      = this->_fsettings["output"]["filesim-out"].get<bool>();
	_interval        = this->_fsettings["output"]["interval"].get<uint32_t>();
	_filesimPrecision = this->_fsettings["output"]["filesim-precision"].get<uint32_t>();
	_filesimSufix    = this->_fsettings["output"]["filesim-sufix"].get<std::string>();
	_filesimPath     = g_baseDir + this->_fsettings["output"]["filesim-path"].get<std::string>();
	_statsOut        = this->_fsettings["output"]["stats-out"].get<bool>();
	_statsInterval   = this->_fsettings["output"]["stats-interval"].get<uint32_t>();
	_statsPath       = g_baseDir + this->_fsettings["output"]["stats-path"].get<std::string>();


	// Asignación de variables globales del proyecto
	g_showProgressBar     = this->_fsettings["output"]["progressBar"].get<bool>();
	g_showTimeExec        = this->_fsettings["output"]["showTimeExec"].get<bool>();
	g_closeEnough         = this->_fsettings["closeEnough"].get<float>();
	g_randomWalkwayRadius = this->_fsettings["randomWalkwayRadius"].get<float>();
	g_attractionRadius    = this->_fsettings["attractionRadius"].get<float>();


	//Tamaño del cuadrante
	uint32_t quadSize = this->_fsettings["quadSize"].get<uint32_t>(); //quadSize[m] x quadSize[m]

	//Se crea el ambiente vacío.
	_env = std::make_shared<Environment>();

	_env->setReferencePoint(_fmap_zone);
	_env->setRouter(_map_osrm);
	_env->setProjector();
	_env->setReferenceZones(_freference_zones);
	_env->setInitialZones(_finitial_zones);

	_env->setGrid(_fmap_zone, quadSize);
	_env->showGrid();

	// inicializar la variable estática _myEnv
	// de la clase Agent
	Agent::_myEnv = _env;

	std::uniform_int_distribution<uint32_t> zone(0, _env->getInitialZones().size() - 1);

	std::cout << "Creando agentes..." << std::endl;
	g_AgentsMem = getMaxMemory();

	auto start = std::chrono::system_clock::now(); //Measure Time

	uint32_t id = 0;
	ProgressBar pg;
	for(auto& fagent : _fsettings["agents"]) {
		uint32_t totalAgents = uint32_t(fagent["number"]);
		pg.start(totalAgents);

		for(uint32_t i = 0; i < totalAgents; i++) {
			if(g_showProgressBar) {
				pg.update(i);
			}

			Point2D position = _env->getInitialZone(zone(rng)).generate();

			json SocialForceModel = fagent["SFM"];

			std::string modelName = fagent["model"].get<std::string>();
			model_t modelID       = model_map[modelName];

			_env->addAgent(\
			               new Agent(id++,\
			                         position,\
			                         fagent["speed"]["min"],\
			                         fagent["speed"]["max"],\
			                         SocialForceModel,\
			                         modelID\
			                        )\
			              );

		}
	}

	auto end = std::chrono::system_clock::now(); //Measure Time
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	g_timeExecMakeAgents += elapsed.count();

	g_AgentsMem = getMaxMemory() - g_AgentsMem;

	if(g_showProgressBar) {
		std::cout << std::flush;
		std::cout << std::endl;
	}
}

void Simulator::calibrate(void)
{

	//
	// Se realiza un ajuste en la posición inicial de los agentes,
	// para que queden en las calles del mapa.
	//
	std::cout << "Ajustando posición inicial de los agentes..." << std::endl;

	auto start = std::chrono::system_clock::now(); //Measure Time
	_env->adjustAgentsInitialPosition(_calibrationTime);
	_env->updateQuads();

	if(g_showProgressBar) {
		std::cout << std::endl;
	}

	//
	// Se ajustan las reglas de los agentes
	//
	std::cout << "Ajustando reglas de los agentes... " <<  std::endl;
	_env->adjustAgentsRules();


	auto end = std::chrono::system_clock::now(); //Measure Time
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	g_timeExecCal += elapsed.count();

	if(g_showProgressBar) {
		std::cout << std::endl;
	}
}

void Simulator::run()
{
	std::cout << "Simulando..." << std::endl;

	g_epochInitSim = std::chrono::duration_cast<std::chrono::seconds>
	                 (std::chrono::system_clock::now().time_since_epoch()).count();

	ProgressBar pg;
	pg.start(_duration-1);

	for(g_currTimeSim = 0; g_currTimeSim < _duration; g_currTimeSim++) {

		if(g_showProgressBar) {
			pg.update(g_currTimeSim);
		}

		if(_saveToDisk && ((g_currTimeSim % _interval) == 0)) {
			this->savePositionAgents();
		}

		auto start = std::chrono::high_resolution_clock::now(); //Measure Time

		_env->updateAgents();
		_env->updateQuads();

		auto end = std::chrono::high_resolution_clock::now(); //Measure Time
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		g_timeExecSim += elapsed.count();
		
		if(_statsOut && ((g_currTimeSim % _statsInterval) == 0)) {
			_env->updateStats();
			this->saveStats();
		}
		//std::cout << " ==> " << elapsed.count() << std::endl;
	}
	
	if(g_showProgressBar) {
		std::cout << std::endl;
	}
	
	if( g_showTimeExec ){
		this->showTimeExec();
	}

	if( _statsOut) {
		std::string pathFile = _statsPath + "/zonesDensity.txt" ;
		std::ofstream ofs(pathFile);
		for(auto& item : g_logZonesDensity) {
			ofs << item << std::endl;
		}
	}
}

Simulator::~Simulator(void)
{
	;
}
void Simulator::savePositionAgents()
{
	std::ostringstream ss;
	ss << std::setw( 10 ) << std::setfill( '0' ) << g_currTimeSim;

	std::string nameFile = ss.str() + "." + _filesimSufix ;
	std::string pathFile = _filesimPath + "/" + nameFile ;

	std::ofstream ofs(pathFile);

	if(_filesimSufix == "txt") {
		for( auto& agent : _env->getAgents() ) {
			double latitude,longitude,h;
			_env->getProjector().Reverse(agent->position()[0],agent->position()[1],0,latitude,longitude,h);
			ofs << agent->id() << " "
			    << std::fixed << std::setprecision(_filesimPrecision) << latitude << " " << longitude
			    << " " << agent->model()
			    << std::endl;
		}
	} else if(_filesimSufix == "geojson") {
		uint32_t currEpoch = g_epochInitSim + g_currTimeSim * 1000;
		ofs << "[";
		for( auto& agent : _env->getAgents() ) {
			double latitude,longitude,h;
			_env->getProjector().Reverse(agent->position()[0],agent->position()[1],0,latitude,longitude,h);
			ofs << "{"
			    << "\"time\": " <<  currEpoch << ", "
			    << "\"id\": "   << agent->id() << ", "
			    << "\"coordinates\": [" << std::fixed << std::setprecision(_filesimPrecision) << longitude << ", " << latitude << "]"
			    << "},";
		}

		ofs << "{ \"time\": " << currEpoch << ", "
		    << "\"id\": "   << -1 << ", "
		    << "\"coordinates\": [0, 0]}]"
		    << std::endl;
	}



}

void Simulator::saveStats()
{
	for(auto& reference_zone : _env->getReferenceZones() ) {
		std::string logString;
		logString = reference_zone.getNameID() + ":" +  \
		            std::to_string(g_currTimeSim) + ":" + \
		            std::to_string(reference_zone.getTotalAgents()) + ":" + \
		            std::to_string(reference_zone.getAgentsDensity());
		g_logZonesDensity.push_back(logString);
	}
}

void Simulator::showTimeExec(void)
{

	uint32_t maxMemory = getMaxMemory();

	std::cout << _duration << ":"
	          << _calibrationTime << ":"
	          << _fsettings["agents"][0]["number"] << ":"
	          << _fsettings["agents"][1]["number"] << ":"
	          << _fsettings["agents"][2]["number"] << ":"
	          << g_timeExecMakeAgents  << ":"
	          << g_timeExecCal << ":"
	          << g_timeExecSim << ":"
	          << g_timeExecSimQuad << ":"
	          << maxMemory << ":"
	          << g_AgentsMem
	          << std::endl;
}


