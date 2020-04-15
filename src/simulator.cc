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

Simulator::Simulator(const json &fsettings,const json &finitial_zones,const json &freference_zones, const json &fmap_zone,const std::string &map_osrm)
{
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());

	std::vector<Agent> _agents;

	_fsettings = fsettings;
	
	// Asignación de variables globales del proyecto
	g_showProgressBar     = _fsettings["output"]["progressBar"].get<bool>();
	g_showTimeExec        = _fsettings["output"]["showTimeExec"].get<bool>();
	g_closeEnough         = _fsettings["closeEnough"].get<float>();
	g_randomWalkwayRadius = _fsettings["randomWalkwayRadius"].get<float>();
	g_attractionRadius    = _fsettings["attractionRadius"].get<float>();
	g_deltaT              = _fsettings["deltaT"].get<float>();

	_duration        = (uint32_t)(_fsettings["duration"].get<uint32_t>() / g_deltaT);
	_calibrationTime = _fsettings["calibration"].get<uint32_t>();
	
	std::string outputBaseDir = _fsettings["output"]["directory"].get<std::string>() + "/";
	
	_saveToDisk      = _fsettings["output"]["agents-out"].get<bool>();
	_interval        = _fsettings["output"]["interval"].get<uint32_t>();
	
	_filesimPrecision = _fsettings["output"]["agents-precision"].get<uint32_t>();
	_filesimSufix    = _fsettings["output"]["agents-sufix"].get<std::string>();
	_filesim         = _fsettings["output"]["agents-path"].get<std::string>();
	_filesimPath     = g_baseDir + outputBaseDir + _filesim;
	
	_statsOut        = _fsettings["output"]["stats-out"].get<bool>();
	_statsInterval   = _fsettings["output"]["stats-interval"].get<uint32_t>();
	_statsPath       = g_baseDir + outputBaseDir +  _fsettings["output"]["stats-path"].get<std::string>();

	_animConfig      = g_baseDir + outputBaseDir + _fsettings["output"]["anim-config"].get<std::string>();

	//Tamaño del cuadrante
	uint32_t quadSize = _fsettings["quadSize"].get<uint32_t>(); //quadSize[m] x quadSize[m]

	//Se crea el ambiente vacío.
	_env = std::make_shared<Environment>();

	_env->setReferencePoint(fmap_zone);
	_env->setRouter(map_osrm);
	_env->setProjector();
	_env->setReferenceZones(freference_zones);
	_env->setInitialZones(finitial_zones);

	_env->setGrid(fmap_zone, quadSize);
	_env->showGrid();

	// inicializar la variable estática _myEnv
	// de la clase Agent y PatchAgent
	Agent::_myEnv = _env;
	PatchAgent::_myEnv = _env; //NEW

	std::cout << "Creando patch agentes..." << std::endl; //NEW
	Environment::grid_t gridData = _env->getGrid();
	for(size_t y = 0; y < gridData._quadY; y++) {
		for(size_t x = 0; x < gridData._quadX; x++) {
			uint32_t idPatch = x + y * gridData._quadX;
			_env->addPatchAgent( new PatchAgent(idPatch) );
		}
	}


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

			auto zoneInitial = _env->getInitialZone(zone(rng));
			//Point2D position = _env->getInitialZone(zone(rng)).generate();
			Point2D position = zoneInitial.generate();
			
			//bool isInside = zoneInitial.pointIsInside(position);
			//std::cout << "Agente:"<< i << ": " << isInside << std::endl;

			json ageRange         = fagent["ageRange"];
			json SocialForceModel = fagent["SFM"];
			json responseTime     = fagent["responseTime"];
			json phoneUse         = fagent["phoneUse"];

			std::string modelName = fagent["model"].get<std::string>();
			model_t modelID       = model_map[modelName];

			_env->addAgent(\
			               new Agent(id++,\
			                         position,\
									 modelID,\
									 ageRange,\
									 phoneUse,\
			                         SocialForceModel,\
									 responseTime
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
	g_logUsePhone.resize(_duration+1);
	
	std::cout << "Simulando..." << std::endl;

	g_epochInitSim = std::chrono::duration_cast<std::chrono::seconds>
	                 (std::chrono::system_clock::now().time_since_epoch()).count();

	ProgressBar pg;
	pg.start(_duration-1);
	
	//Tiempo 0 equivale a las posiciones iniciales
	g_currTimeSim = 0;
	this->savePositionAgents();

	for(g_currTimeSim = 1; g_currTimeSim <= _duration; g_currTimeSim++) {

		if(g_showProgressBar) {
			pg.update(g_currTimeSim);
		}

		/*if(_saveToDisk && ((g_currTimeSim % _interval) == 0)) {
			this->savePositionAgents();
		}*/

		auto start = std::chrono::high_resolution_clock::now(); //Measure Time

		_env->updateAgents();
		_env->updateQuads();

		auto end = std::chrono::high_resolution_clock::now(); //Measure Time
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		g_timeExecSim += elapsed.count();
		
		
		if(_saveToDisk && ((g_currTimeSim % _interval) == 0)) {
			this->savePositionAgents();
		}

		if(_statsOut && ((g_currTimeSim % _statsInterval) == 0)) {
			_env->updateStats();
			this->saveStats();
		}
		//std::cout << " ==> " << elapsed.count() << std::endl;
	}

	if(g_showProgressBar) {
		std::cout << std::endl;
	}

	if( g_showTimeExec ) {
		this->showTimeExec();
	}

	if( _statsOut) {
		std::string pathFile01;
		std::string pathFile02;
		std::string pathFile03;
		
		pathFile01 = _statsPath + "/zonesDensity.txt" ;
		std::ofstream ofs01(pathFile01);
		for(auto& item : g_logZonesDensity) {
			ofs01 << item << std::endl;
		}
		ofs01.close();
		
		pathFile01 = _statsPath + "/usePhone.txt" ;
		ofs01.open(pathFile01);
		
		for(uint32_t tick = 0; tick <= _duration; tick += _statsInterval){
			uint32_t timeSim = tick * g_deltaT;
			ofs01 << std::to_string(timeSim) << " " << std::to_string(g_logUsePhone[tick]) << std::endl;
		}
		ofs01.close();
		
		pathFile01 = _statsPath + "/summary.txt" ;
		ofs01.open(pathFile01);

		ofs01 << "id model groupAge safeZone distanceToTargetPos responseTime evacTime" << std::endl;		
		for(auto& fooAgent : _env->getAgents()){
			ofs01 << std::to_string(fooAgent->id()) << " "  
				<< fooAgent->model() << " " 
				<< std::to_string(fooAgent->groupAge()) << " " 
				<< fooAgent->getSafeZoneID() << " " 
				<< std::to_string(fooAgent->distanceToTargetPos()) << " " 
				<< std::to_string(fooAgent->responseTime()) << " " 
				<< std::to_string(fooAgent->evacuationTime()) << std::endl;	
		}
		ofs01.close();
		
	}
	
	if(_saveToDisk){
		//Crea el archivo JSON de la animación de la simulación
		json animacionConfig = {
			{"pathFileSim", "agents/"},
			{"frameMin", 0},
			{"frameMax", _duration},
			{"frameStep", _interval},
			{"deltaT", g_deltaT},
			{"input",{
				{"area"            , "input/area.geojson"},
				{"initial_zones"   , "input/initial_zones.geojson"},
				{"reference_zones" , "input/reference_zones.geojson"}
			}}
		};
		
		//std::ofstream ofs(_fsettings["output"]["anim-config"].get<std::string>()); _animConfig
		std::ofstream ofs( _animConfig );
		
		ofs << animacionConfig.dump(4) << std::endl;
		
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
				<< " " << agent->getNextTimeUsePhone()
				<< " " << agent->getProbUsePhone()
				<< " " << agent->getAgentNeighborsSize()
				<< " " << agent->getUsingPhone()
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
	/*
	for(auto& reference_zone : _env->getReferenceZones() ) {
		std::string logString;
		logString = reference_zone.getNameID() + ":" +  \
		            std::to_string(g_currTimeSim) + ":" + \
		            std::to_string(reference_zone.getTotalAgents()) + ":" + \
		            std::to_string(reference_zone.getAgentsDensity());
		g_logZonesDensity.push_back(logString);
	}
	*/
	
	std::string logString;
	
	logString = std::to_string(g_currTimeSim);
	
	for(auto& reference_zone : _env->getReferenceZones() ) {	
		logString +=  ":" + reference_zone.getNameID() + ":" +  \
		            std::to_string(reference_zone.getTotalAgents()) + ":" + \
		            std::to_string(reference_zone.getAgentsDensity());		
	}
	g_logZonesDensity.push_back(logString);
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
	          << maxMemory << ":"
	          << g_AgentsMem
	          << std::endl;
}


