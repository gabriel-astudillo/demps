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
	g_closeEnough         = _fsettings["closeEnough"].get<float>();
	g_randomWalkwayRadius = _fsettings["randomWalkwayRadius"].get<float>();
	g_attractionRadius    = _fsettings["attractionRadius"].get<float>();
	g_deltaT              = _fsettings["deltaT"].get<float>();
	
	_numExperiment = _fsettings["numExperiment"].get<int32_t>();

	_duration        = (uint32_t)(_fsettings["duration"].get<uint32_t>() / g_deltaT);
	_calibrationTime = (uint32_t)(_fsettings["calibration"].get<uint32_t>() / g_deltaT);
	
	std::string outputBaseDir = _fsettings["output"]["directory"].get<std::string>() + "/";
	
	_saveToDisk      = _fsettings["output"]["agents-out"].get<bool>();
	_interval        = (uint32_t)(_fsettings["output"]["interval"].get<uint32_t>() /  g_deltaT);
	
	_filesimPrecision = _fsettings["output"]["agents-precision"].get<uint32_t>();
	_filesimSufix    = _fsettings["output"]["agents-sufix"].get<std::string>();
	_filesim         = _fsettings["output"]["agents-path"].get<std::string>();
	_filesimPath     = g_baseDir + outputBaseDir + _filesim;
	
	_statsOut        = _fsettings["output"]["stats-out"].get<bool>();
	_statsInterval   = (uint32_t)(_fsettings["output"]["stats-interval"].get<uint32_t>() /  g_deltaT);
	
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
	PatchAgent::_myEnv = _env; 

	std::cout << "Creando patch agentes..." << std::endl; 
	Environment::grid_t gridData = _env->getGrid();
	for(size_t y = 0; y < gridData._quadY; y++) {
		for(size_t x = 0; x < gridData._quadX; x++) {
			uint32_t idPatch = x + y * gridData._quadX;
			_env->addPatchAgent( new PatchAgent(idPatch) );
		}
	}


	std::uniform_int_distribution<uint32_t> zone(0, _env->getInitialZones().size() - 1);
	
	std::uniform_real_distribution<double> unif(0.0, 1.0);
	std::vector<double> initZoneAreasAcum;
	double areaTotal = 0.0;
	
	for(auto& fooInitZone : _env->getInitialZones()){
		initZoneAreasAcum.push_back(fooInitZone.getArea());
		areaTotal += fooInitZone.getArea();
	}
	
	for(size_t i = 0; i < initZoneAreasAcum.size(); ++i){
		if(i == 0){
			initZoneAreasAcum[i] /= areaTotal;
		}
		else{
			initZoneAreasAcum[i] /= areaTotal;
			initZoneAreasAcum[i] += initZoneAreasAcum[i-1];
		}
	}

	
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
				
			//La probabilidad de seleccion de la zona inicial
			//depende del area de ella
			/*
			uint32_t initialZoneIndex = 0;
			double trigg = unif(rng);
			for(size_t i = 0; i < initZoneAreasAcum.size(); ++i){			
				if(trigg <= initZoneAreasAcum[i]){
					initialZoneIndex = i;
					break;
				}
			}
			auto initialZone = _env->getInitialZone(initialZoneIndex);
			*/

			//La probabilidad de seleccion de la zona inicial
			//es uniforme
			auto initialZone = _env->getInitialZone(zone(rng));
			
			Point2D position = initialZone.generate();
			std::string initialZoneNameID = initialZone.getNameID();
			
			json ageRange         = fagent["ageRange"];
			json SocialForceModel = fagent["SFM"];
			json responseTime     = fagent["responseTime"];
			json phoneUse         = fagent["phoneUse"];

			std::string modelName = fagent["model"].get<std::string>();
			model_t modelID       = model_map[modelName];

			_env->addAgent(\
			               new Agent(id++,\
			                         position,\
									 initialZoneNameID,\
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
	g_logVelocity.resize(_env->getTotalAgents());
	
	std::cout << "Simulando..." << std::endl;

	g_epochInitSim = std::chrono::duration_cast<std::chrono::seconds>
	                 (std::chrono::system_clock::now().time_since_epoch()).count();

	ProgressBar pg;
	pg.start(_duration-1);
	
	//Tiempo 0 equivale a las posiciones iniciales
	g_currTimeSim = 0;
	
	if( _saveToDisk ) {
		this->savePositionAgents();
	}

	for(g_currTimeSim = 1; g_currTimeSim <= _duration; g_currTimeSim++) {

		if(g_showProgressBar) {
			pg.update(g_currTimeSim);
		}

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

	}

	if(g_showProgressBar) {
		std::cout << std::endl;
	}

	this->executionSummary();

	if( _statsOut) {
		std::string pathFile01;
		std::string pathFile02;
		
		
		std::string nameSuffix = this->getNameSuffix();
		
		/*
		**	zonesDensity
		*/
		pathFile01 = _statsPath + "/zonesDensity" +  nameSuffix;
		std::ofstream ofs01(pathFile01);
		for(auto& item : g_logZonesDensity) {
			
			if( _numExperiment >= 0){
				ofs01 << _numExperiment << ":";
			}
			
			ofs01 << item << std::endl;
		}
		ofs01.close();
		
		/*
		**	usePhone
		*/
		pathFile01 = _statsPath + "/usePhone" + nameSuffix ;
		ofs01.open(pathFile01);
			
		
		if( _numExperiment >= 0){
			ofs01 << "numExperiment:" ;
		}
		
		ofs01 << "timeStamp:usePhone" << std::endl;		
		
		
		for(uint32_t tick = 0; tick <= _duration; tick += _statsInterval){
			uint32_t timeSim = tick * g_deltaT;
			
			if( _numExperiment >= 0){
				ofs01 << _numExperiment << ":";
			}
			
			ofs01 << std::to_string(timeSim) << ":" << std::to_string(g_logUsePhone[tick]) << std::endl;
		}
		ofs01.close();
		
		/*
		**	velocity
		*/
		pathFile01 = _statsPath + "/velocity" + nameSuffix ;
		ofs01.open(pathFile01);
		
		if( _numExperiment >= 0){
			ofs01 << "numExperiment:" ;
		}
		
		ofs01 << "id:model:groupAge:";
		for(uint32_t tick = 0; tick <= _duration; tick += _statsInterval){
			uint32_t timeSim = tick * g_deltaT;
		
			ofs01 << std::to_string(timeSim) << ":";
		}
		
		ofs01 << std::endl;
		
		ofs01 << "Coming soon..." <<std::endl;
		
		/*
		for(auto& fooAgent : _env->getAgents()){
			ofs01 << std::to_string(fooAgent->id()) << ":";
			ofs01 << std::to_string(fooAgent->model()) << ":" ;
			ofs01 << std::to_string(fooAgent->groupAge()) << ":";
			ofs01 << g_logVelocity[fooAgent->id()] << std::endl;
		}
		*/
		
		ofs01.close();
		
		/*
		**	summary
		*/
		pathFile01 = _statsPath + "/summary" + nameSuffix ;
		ofs01.open(pathFile01);
		
		if( _numExperiment >= 0){
			ofs01 << "numExperiment:" ;
		}
		
		std::random_device _randomDevice;
		//std::uniform_real_distribution<> unifDistro(-(float)_interval/2.0, (float)_interval/2.0);	 
		std::uniform_real_distribution<> unifDistro(-(float)_interval*g_deltaT/2.0, (float)_interval*g_deltaT/2.0); //2020-10-09

		ofs01 << "id:model:groupAge:safeZone:distanceToTargetPos:responseTime:evacTime:initialZone" << std::endl;		
		for(auto& fooAgent : _env->getAgents()){
			if( _numExperiment >= 0){
				ofs01 << _numExperiment << ":";
			}
			
			//aleatorizar el tiempo de evacuación para que esté 
			//en un rango tevac +/- (_interval)/2
			double evacTime = fooAgent->evacuationTime();
			//if( fooAgent->evacuationTime() > 0 ){
			//if( fooAgent->evacuationTime() > (fooAgent->responseTime() + _interval/2.0) ){
			//if( fooAgent->evacuationTime() >  (float)_interval/2.0 ){
			if( fooAgent->evacuationTime() >  (float)_interval*g_deltaT/2.0 ){ //2020-10-09
				evacTime +=  unifDistro(_randomDevice) ; 
				
				/*if(evacTime < fooAgent->responseTime() ){
					evacTime = fooAgent->evacuationTime();
					evacTime +=  std::abs(unifDistro(_randomDevice)) ; 
				}*/
			}
			
			
			//Para todos los visitantes II que no alcanzaron a evacuar
			//debido a que nunca supuieron donde estaba su zona segura
			//más cercana, determinar su zona segura y calcula a qué
			//distancia quedaron.
			if(fooAgent->safeZone() == nullptr && fooAgent->model() == Visitors_II){
				_env->setSafeZoneAttribAgent(fooAgent);
			}
			
			
			ofs01 << std::to_string(fooAgent->id()) << ":"  
				<< fooAgent->model() << ":" 
				<< std::to_string(fooAgent->groupAge()) << ":" 
				<< fooAgent->getSafeZoneID() << ":" 
				<< std::to_string(fooAgent->distanceToTargetPos()) << ":" 
				<< std::to_string(fooAgent->responseTime()) << ":" 
				<< std::to_string(evacTime) << ":"
				<< fooAgent->getInitialZoneID()
				<< std::endl;	
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
				//<< " " << agent->getNextTimeUsePhone()
				//<< " " << agent->getProbUsePhone()
				//<< " " << agent->getAgentNeighborsSize()
				//<< " " << agent->getDensity()
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
	std::string logString;
	
	logString = std::to_string(g_currTimeSim*g_deltaT);
	
	for(auto& reference_zone : _env->getReferenceZones() ) {	
		logString +=  ":" + reference_zone.getNameID() + ":" +  \
		            std::to_string(reference_zone.getTotalAgents()) + ":" + \
		            std::to_string(reference_zone.getAgentsDensity());		
	}
	g_logZonesDensity.push_back(logString);
}

void Simulator::executionSummary()
{
	std::string pathFile01;
	
	uint32_t maxMemory = getMaxMemory();
	
	std::string nameSuffix = this->getNameSuffix();
	
	pathFile01 = _statsPath + "/executionSummary" + nameSuffix ;
	std::ofstream ofs01(pathFile01);
	
	if( _numExperiment >= 0){
		ofs01 << "numExperiment:";
	}

	ofs01 << "tsim:calibrationTime:deltaT:Residents:Visitors:timeExecMakeAgents:timeExecCal:timeExecSim:maxMemory:agentsMem" << std::endl;	
	
	if( _numExperiment >= 0){
		ofs01 << _numExperiment << ":";
	}
	
	ofs01 << _duration*g_deltaT << ":"
	      << _calibrationTime*g_deltaT << ":"
		  << g_deltaT << ":"
	      << _fsettings["agents"][0]["number"] << ":"
		  << _fsettings["agents"][1]["number"] << ":"
	      << g_timeExecMakeAgents  << ":"
	      << g_timeExecCal << ":"
	      << g_timeExecSim << ":"
	      << maxMemory << ":"
	      << g_AgentsMem
	      << std::endl;
	
	ofs01.close();
}

std::string Simulator::getNameSuffix()
{
	std::string nameSuffix = "";
	
	if( _numExperiment >= 0){
		std::ostringstream ss;
		ss << "-" << std::setw( 5 ) << std::setfill( '0' ) << _numExperiment;
		nameSuffix = ss.str();
	}
	
	nameSuffix += ".txt";
	
	return(nameSuffix);
	
}


