#include <simulator.hh>
#include <environment.hh>
#include <zone.hh>

#include <PGM.hpp>

uint32_t getMaxMemory()
{
	struct rusage r_usage;
	getrusage(RUSAGE_SELF,&r_usage);
	//https://www.gnu.org/software/libc/manual/html_node/Resource-Usage.html
	uint32_t maxMemory = r_usage.ru_maxrss;//KB

	return(maxMemory);
}

std::mutex Simulator::_execForMTX;
bool Simulator::_simInExec;


Simulator::Simulator(void)
{

}

////////////////////////////////////////////////////////
//	Simulator::simulator()
//
//Simulator::Simulator(const json &fsettings, const json& fzones, const json& finitial_zones, const json& freference_zones, const json& fmap_zone,const std::string &map_osrm)
//Simulator::Simulator(const json &fsettings, const json& fzones, const json& fmap_zone,const std::string &map_osrm)
Simulator::Simulator(const json &fsettings, const json& fzones, const std::string &map_osrm)
{
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());

	std::vector<Agent> _agents;

	_fsettings = fsettings;
	
	//Crear el identificador único de la simulación
	_uuidSim = utils::get_uuid();
	
	// Asignación de variables globales del proyecto
	g_showProgressBar     = _fsettings["output"]["progressBar"].get<bool>();
	g_panicModelEnable    = _fsettings["panicModelEnable"].get<bool>(); 
	g_elevationModelEnable = _fsettings["elevationModelEnable"].get<bool>();
	g_elevationPatchDataValid = _fsettings["elevationPatchDataValid"].get<bool>();
	g_debrisModelEnable   = _fsettings["debrisModelEnable"].get<bool>();
	g_floodModelEnable    = _fsettings["floodModelEnable"].get<bool>();
	g_closeEnough         = _fsettings["closeEnough"].get<float>();
	g_randomWalkwayRadius = _fsettings["randomWalkwayRadius"].get<float>();
	g_attractionRadius    = _fsettings["attractionRadius"].get<float>();
	g_deltaT              = _fsettings["deltaT"].get<float>();
	
	_numExperiment = _fsettings["numExperiment"].get<int32_t>();

	_duration        = (uint32_t)(_fsettings["duration"].get<uint32_t>() / g_deltaT);
	_calibrationTime = (uint32_t)(_fsettings["calibration"].get<uint32_t>() / g_deltaT);
	
	std::string outputBaseDir = _fsettings["output"]["directory"].get<std::string>() + "/";
	
	_agentsOut      = _fsettings["output"]["agents-out"].get<bool>();
	g_agentsOut     = _agentsOut;
	
	_interval       = (uint32_t)(_fsettings["output"]["interval"].get<uint32_t>() /  g_deltaT);
	
	_filesimPrecision = _fsettings["output"]["agents-precision"].get<uint32_t>();
	_filesimSufix    = _fsettings["output"]["agents-sufix"].get<std::string>();
	_filesimPath     = g_baseDir + outputBaseDir + _fsettings["output"]["agents-path"].get<std::string>();

    _heatMapOut      = _fsettings["output"]["heatMap-out"].get<bool>();
	_heatMapSize     = _fsettings["output"]["heatMap-size"].get<uint32_t>();
	_heatMapInterval = (uint32_t)(_fsettings["output"]["heatMap-interval"].get<uint32_t>() /  g_deltaT);
	
	
	
	std::cout << "\x1B[1;37m";
	/*
	if(_fsettings["output"]["heatMap-path"].get<std::string>()[0] != '/'){
		_heatMapPath = g_baseDir + outputBaseDir + _fsettings["output"]["heatMap-path"].get<std::string>();
		std::cout << "Production Simulation." << std::endl;
		std::cout << "\tThe simulation sample will be compared with others." << std::endl;
		std::cout << "\tThe simulation will not be saved in the SnitchServer." << std::endl;
		_saveSimInDB = false;
		_compareWithOthersSimIsOn = true;
	}
	else{
		// La ruta al directorio de heatMaps es absoluta. 
		//  ==> Los heatMaps se guardan en ese directorio para que futuras 
		//      simulaciones busquen heatMaps parecidos.
		//  ==> Los heatMaps de cada simulacion se diferencian por su uuid
		_heatMapPath = _fsettings["output"]["heatMap-path"].get<std::string>() + "/" + _uuidSim;
		std::cout << "Training Simulation." << std::endl;
		std::cout << "\tThe simulation sample will not be compared with others." << std::endl;
		std::cout << "\tThe simulation sample will be sent to SnitchServer." << std::endl;
			
		_saveSimInDB = true;
		_compareWithOthersSimIsOn = false;
	}
	*/
	// todos los mapas de calor se almacenan para realizar simulaciones aproximadas
	//Los heatMaps de cada simulacion se diferencian por su uuid
	_heatMapPath = g_baseDir + _fsettings["output"]["heatMap-path"].get<std::string>() + "/" + _uuidSim;;
	_saveSimInDB = true;
	_compareWithOthersSimIsOn = true;
	std::cout << "Training and production Simulation.\n";
	std::cout << "\tThe simulation sample will be sent to SnitchServer.\n";
	std::cout << "\tThe simulation sample will be compared with others.\n";
	
	
	if( !std::filesystem::exists(_heatMapPath)){
		std::cout << "Path to heatMap directory " << _heatMapPath << " no exist. Creating." << std::endl;
		std::filesystem::create_directories(_heatMapPath);
	}
	std::cout << "\x1B[0m" << std::endl;
	
	_debrisFilePath = g_baseDir + outputBaseDir + _fsettings["debrisParams"]["stateDir"].get<std::string>();
	if( !std::filesystem::exists( _debrisFilePath )){
		std::cout << "Path to debris directory " << _debrisFilePath << " no exist. Creating." << std::endl;
		std::filesystem::create_directories( _debrisFilePath );
	}
	
	
	_statsOut        = _fsettings["output"]["stats-out"].get<bool>();
	_statsInterval   = (uint32_t)(_fsettings["output"]["stats-interval"].get<uint32_t>() /  g_deltaT);
	_statsPath       = g_baseDir + outputBaseDir +  _fsettings["output"]["stats-path"].get<std::string>();
	
	_samplingInterval = _fsettings["samplingInterval"].get<uint32_t>();

	_animConfig      = g_baseDir + outputBaseDir + _fsettings["output"]["anim-config"].get<std::string>();
	

	//Tamaño del cuadrante
	uint32_t quadSize = _fsettings["quadSize"].get<uint32_t>(); //quadSize[m] x quadSize[m]

	//Se crea el ambiente vacío.
	_env = std::make_shared<Environment>();
	
	//
	// inicializar la variable estática _myEnv
	// en todas las clases
	Agent::_myEnv = _env;
	PatchAgent::_myEnv = _env; 
	Zone::_myEnv = _env;
	//ZoneBasic::_myEnv = _env;
	Router::_myEnv = _env;

	_env->setProjector(fzones);
	_env->setRouter(map_osrm);
	
	uint32_t offsetMap = 500; //meters
	_env->setGrid(fzones, offsetMap, quadSize);
	_env->showGrid();
	
	// Carga parámetros del modelo de densidad
	json densityParams = _fsettings["densityParams"];
	densityParams["enable"] = _fsettings["densityModelEnable"];
	_env->setDensityParams( densityParams );
	
	// Los directorios de imagenes y estado de la inundación se convierten a rutas absolutas si son relativas
	json floodParams;
	floodParams = _fsettings["floodParams"];

	if(floodParams["imagesDir"].get<std::string>()[0] != '/'){
		floodParams["imagesDir"] = g_baseDir + outputBaseDir + floodParams["imagesDir"].get<std::string>();
		if( !std::filesystem::exists(floodParams["imagesDir"].get<std::string>() )){
			std::filesystem::create_directories(floodParams["imagesDir"].get<std::string>());
		}
	}

	if(floodParams["stateDir"].get<std::string>()[0] != '/'){
		floodParams["stateDir"] = g_baseDir + outputBaseDir + floodParams["stateDir"].get<std::string>();
		if( !std::filesystem::exists(floodParams["stateDir"].get<std::string>() )){
			std::filesystem::create_directories(floodParams["stateDir"].get<std::string>());
		}

	}
	
	floodParams["enable"] = _fsettings["floodModelEnable"];
	
	
	_env->setFloodParams( floodParams );
	Environment::floodParams_t simFloodParams = _env->getFloodParams();
	std::cout << "floodParams:"  << std::endl; 
	std::cout << "\tenable               : " << simFloodParams.enable << std::endl;
	std::cout << std::setprecision(2);
	std::cout << "\tarrivalTime     (s)  : " << simFloodParams.arrivalTime << std::endl; 
	std::cout << "\tspeedWaterLevel (m/s): " << simFloodParams.speedWaterLevel << std::endl; 
	std::cout << "\tspeedWaterProp  (m/s): " << simFloodParams.speedWaterProp << std::endl; 
	std::cout << "\tcriticalLevel   (m)  : " << simFloodParams.criticalLevel << std::endl; 
	std::cout << "\tminSpeedFactor  [0,1]: " << simFloodParams.minSpeedFactor << std::endl;
	
	if(g_elevationModelEnable && g_elevationPatchDataValid){
		std::cout << "Cargando datos de elevación de terreno..." << std::endl; 	
		std::string elevationFile = fsettings["input"]["elevationPatchData"].get<std::string>() ; // la ruta completa se completó en demps.cc
		
		std::map<int32_t, std::tuple<double, double,int32_t> > elevationData;
		utils::elevationDataToVector(elevationFile, elevationData);
		
		_env->setElevationData(elevationData);
	
	}
	

	
	///////////////////////////////////////////////////////////////////////////
	//
	// Crear patchs agents
	//
	std::cout << "Creando patch agentes..." << std::endl; 	
	Environment::grid_t gridData = _env->getGrid();

	//#pragma omp parallel for
	for(size_t y = 0; y < gridData._quadY; y++) {
		for(size_t x = 0; x < gridData._quadX; x++) {
			uint32_t idPatch = x + y * gridData._quadX;
			//std::cout << idPatch << std::endl;
			_env->addPatchAgent( new PatchAgent(idPatch) );//, g_elevationModelEnable) );
			
		}
	}

	// Si el argumento patchCoords está presente, se hace el 
	// el volcado de coordenadas de los patch agents en el archivo
	// elevationPatch-<ciudad>.txt
	if(fsettings["patchCoords"]){
		ProgressBar pgElevCoords;
		std::string pathFile = "elevationPatch-"+ fsettings["city"].get<std::string>() + ".txt";
		
		std::cout << "Guardando coordenadas de los patch en '" + pathFile + "'..." << std::endl; 	
		
		pgElevCoords.start(_env->getPatchAgents().size());
		std::ostringstream elevationData;
		for(size_t idAgent = 0; idAgent < _env->getPatchAgents().size(); idAgent++ ){
			if(g_showProgressBar) {
				pgElevCoords.update(idAgent);
			}
			PatchAgent::quad_t pAgentQuad =  _env->getPatchAgent(idAgent)->getQuadInfo();
				
			if(_env->getPatchAgent(idAgent)->haveElevation()){
				elevationData << std::fixed << std::setprecision(_filesimPrecision);
				elevationData << idAgent << ":" << pAgentQuad.lat << ":" << pAgentQuad.lon;
				elevationData << std::endl;
			}
			
		}
		if(g_showProgressBar) {
			std::cout << std::flush;
			std::cout << std::endl;
		}
		
		std::ofstream ofs(pathFile);
		ofs << elevationData.str();
		ofs.close();
		
		exit(0);
	}
	
	////////////////////////////////////////////////////////////////
	// Procesar el archivos de zones y cargar:
	//      a) zonas iniciales y zonas seguras
	//      b) zonas inundables
	//      c) zonas geográficas para monitorear flujo de personas
	//          c.1) zoneType == "lineMonitor"
	//          c.2) zoneType == "pointMonitor"
	//
	std::cout << "\x1B[0;90m";
	std::cout << "Revisando mapa:" << std::endl;
	for(const auto& feature : fzones["features"]) {
		std::string zoneType = feature["properties"]["zoneType"].get<std::string>();
		std::string nameID   = feature["properties"]["nameID"].get<std::string>();
		//std::cout << "zoneType:" << zoneType << std::endl;
		if(zoneType == "initial"){
			std::cout << "\tCreando zona inicial: " << nameID << std::endl; 
			_env->addInitialZone(feature);
		}
		else if(zoneType == "safe"){
			std::cout << "\tCreando zona segura: " << nameID << std::endl; 
			_env->addReferenceZone(feature);
		}
		else if(zoneType == "flood" && _env->getFloodParams().enable){
			std::cout << "\tCreando zona inundable: " << nameID << std::endl; 
			_env->addFloodZone(feature);
		}
		else if(zoneType == "lineMonitor"){
			std::cout << "\tAsignando patch agentes monitores en línea geográfica: " << nameID << std::endl; 
			_env->addLineMonitorZone(feature);
		}
		else if(zoneType == "\tpointMonitor"){
			std::cout << "Asignando patch agentes monitores en punto geográfico: " << nameID << std::endl; 
			_env->addPointMonitorZone(feature);
		}
	}
	std::cout << "\x1B[0m" << std::endl;
	
	////////////////////////////////////////////
	// Si no hay zonas inundables definidas,
	// implica que el modelo de inundación no
	// debe estar habilitado.
	if(_env->getFloodZones().size() == 0){
		floodParams["enable"] = false;
		_env->setFloodParams( floodParams );
	}
	
	if(_env->getFloodParams().enable){
		// Por cada zona de inundación, determinar los
		// patch agents que le corresponden.
		std::cout << "\x1B[0;90m";
		std::cout << "Configurando zonas inundables:" << std::endl;
		std::cout << "\tasignando patch agents." << std::endl;
		_env->assignPatchAgentsToFloodZones();
		std::cout << "\tdeterminando coordenadas patch agents perimetrales." << std::endl;
		_env->setNSWEPatchAgentsAllZones();
		
		for(auto& floodZone : _env->getFloodZones()) {
			std::cout << "Zona inundable: " << floodZone.getNameID() << std::endl;
			std::cout << "\tarea:" << floodZone.getArea() << std::endl;
			std::cout << "\ttotal Patchs: " << floodZone.patchAgentsInZone().size() << std::endl;
			std::cout << "\tmax level flood (m): " << floodZone.getMaxLevelFlood() << std::endl;
		}
		
		std::cout << "Identificadores idX,idY de patchAgent de referencia: " << std::endl;
		Zone::NSWEPatchAgentsAllZone_t idsPatchAgents = _env->getNSWEPatchAgentsAllZones();
		std::cout << "\tmost further North(idY max) : " << idsPatchAgents[0] << std::endl;
		std::cout << "\tmost further South (idY min): " << idsPatchAgents[1] << std::endl;
		std::cout << "\tmost further West (idX min) : " << idsPatchAgents[2] << std::endl;
		std::cout << "\tmost further East (idX max) : " << idsPatchAgents[3] << std::endl;
		
		std::cout << "\x1B[0m" << std::endl;
	}
	
	
	PatchAgent::monitorGroup pointMonitorGroups, lineMonitorGroups;
	
	pointMonitorGroups = _env->getMonitorPatchAgentGroups(PatchAgent::typeMonitor::pointMonitor);
	lineMonitorGroups  = _env->getMonitorPatchAgentGroups(PatchAgent::typeMonitor::lineMonitor);
	
	/*
	// Sólo para ver cuáles son los patchAgents que corresponden
	// a los pointMonitor y lineMonitor	
	for(const auto& [idGroup, patchAgents]: pointMonitorGroups){
		std::cout << "pointMonitorGroupID: " << idGroup << std::endl;
		
		for(const auto& pAgent: patchAgents){
			uint32_t quadId = pAgent->getId();
			PatchAgent::quad_t quadInfo = pAgent->getQuadInfo();
		
			std::cout << "xQuad=" << quadInfo.idX << ", yQuad=" << quadInfo.idY;
			std::cout << "\t==> quadID=" << quadId;
			std::cout << std::endl;
		}
	}
	for(const auto& [idGroup, patchAgents]: lineMonitorGroups){
		std::cout << "lineMonitorGroupID: " << idGroup << std::endl;
		
		for(const auto& pAgent: patchAgents){
			uint32_t quadId = pAgent->getId();
			PatchAgent::quad_t quadInfo = pAgent->getQuadInfo();
		
			std::cout << "xQuad=" << quadInfo.idX << ", yQuad=" << quadInfo.idY;
			std::cout << "\t==> quadID=" << quadId;
			std::cout << std::endl;
		}
	}
	*/
	
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
			auto& initialZone = _env->getInitialZone(zone(rng));	
			Point2D position = initialZone.generate();
			std::string initialZoneNameID = initialZone.getNameID();
			
			//std::cout << std::setprecision(_filesimPrecision);
			//std::cout << "##"<< initialZoneNameID  <<": "<< initialZone.getCentroidWGS84();
			//std::cout <<  ": " << initialZone.getCentroid() <<std::endl;
			
			
			json ageRange         = fagent["ageRange"];
			json SocialForceModel = fagent["SFM"];
			json responseTime     = fagent["responseTime"];
			json phoneUse         = fagent["phoneUse"];
			json panicModel       = fagent["panicModel"];
			if(fsettings["panicModelEnable"] == true && fsettings["panicModelManageByParameter"] == true){
				// El nivel minimo de emoción para que el agente se infecte se carga desde los 
				// parámetros del programa (L:120 demps.cc)
				panicModel["emotionThreshold"] = fsettings["emotionThreshold"].get<double>() / 100.0;
			}
			

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
									 panicModel,\
									 responseTime
			                        )\
			              );

		
			
			// Debug para ver los niveles iniciales de pánico.
			if(i==10){
				_env->getAgent(i)->showPanic();
			}
			
		}
	}
	g_totalAgentsInSim = id;
	auto end = std::chrono::system_clock::now(); //Measure Time
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	g_timeExecMakeAgents += elapsed.count();

	g_AgentsMem = getMaxMemory() - g_AgentsMem;

	if(g_showProgressBar) {
		std::cout << std::flush;
		std::cout << std::endl;
	}
	
}

////////////////////////////////////////////////////////
//	Simulator::calibrate()
//
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
	// Determinar los patch agents que están en la calles donde
	// transitan los agentes
	//
	std::cout << "Determinando patch agents que contienen calles... " <<  std::endl;
	_env->determinatePAgentsInStreets();
	
	if(g_showProgressBar) {
		std::cout << std::endl;
	}
	
	//
	// Determinar proporcion de los patchs agentes que están en las
	// calles que deben tener escombros;
	//
	
	if(g_debrisModelEnable){
		double debrisRatio = _fsettings["debrisParams"]["debrisRatio"].get<double>()/100.0;
		_env->determinatePAgentsWithDebris(debrisRatio);
	}
	
	
	
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
	

	/*
	for(auto &fooZone : _env->getInitialZones()) {
		std::cout << std::setprecision(_filesimPrecision);
		std::cout << "ZONA " << fooZone.getNameID() << "\n" ;
		std::cout << "Area: " <<fooZone.getArea() << "\t\tcentroidWGS84:" << fooZone.getCentroidWGS84() << "\n";
		std::cout << "fShape: " << fooZone.getshapeForm() << "\trEquiv: " << fooZone.getRequiv() << std::endl;
	}

	for(auto &fooZone : _env->getReferenceZones()) {
		std::cout << std::setprecision(_filesimPrecision);
		std::cout << "ZONA " << fooZone.getNameID() << "\n" ;
		std::cout << "Area: " <<fooZone.getArea() << "\t\tcentroidWGS84:" << fooZone.getCentroidWGS84() << "\n";
		std::cout << "fShape: " << fooZone.getshapeForm() << "\trEquiv: " << fooZone.getRequiv() << std::endl;
	}
	
	
	Zone ZA =_env->getReferenceZone(0);
	Zone ZB =_env->getReferenceZone(1);
	double jIndex = zonesSimilarity::jaccardIndex(ZA, ZB);
	std::cout << "ZA -> ZB :" << jIndex << std::endl;
	*/

	if(g_showProgressBar) {
		std::cout << std::endl;
	}
	
	//exit(EXIT_SUCCESS);
}

////////////////////////////////////////////////////////
//	Simulator::run()
//
void Simulator::run()
{
	std::string restURL = "http://127.0.0.1:6502/v1/snitch/api";
	json        similarSims;
	
	utils::radius_t seekRadius = 0.5; 
	uint32_t        xsteps     = 10;
	uint32_t        cutOff     = 10;
	utils::radius_t seekRadiusHMap = 5;
	uint32_t        cutoffHMap     = 10;
	
	_samplingLevel = _fsettings["samplingLevel"];
	
	g_logUsePhone.resize(_duration+1);
	g_logVelocity.resize(_env->getTotalAgents());
	
	////////////////////////////////////////////////////////////////////////
	// Características de las zonas iniciales y seguras
	//
	json sdata_zonesInfo;
	_env->getZonesInfo(sdata_zonesInfo);
	
	std::cout << "Simulando..." << std::endl;
	
	uint32_t initialWaitTime = 60; //segundos
	uint32_t deltaTime       = 60; //segundos
	uint32_t thresTime       = 120; //segundos
	std::string dirTodDelete = _heatMapPath;
	_simInExec = true;
	std::thread watchDogThread(watchDog, initialWaitTime, deltaTime, thresTime, dirTodDelete);
	
	
	ProgressBar pg;
	pg.start(_duration-1);
	std::cout << "Simulando... tick=0" << std::endl;
	//////////////////////////////////////////////
	//Tiempo 0 equivale a las posiciones iniciales
	g_currTimeSim = 0;
	
	if( _agentsOut ) {
		this->savePositionAgents();
	}

	_env->updateAgents();
	_env->updateQuads();
	
	if( _heatMapOut ){
		this->saveStatePatchAgents();
	}
	
	if(_env->getFloodParams().enable){
		// crea imagen PGM del estado inicial de la inundación
		this->saveStateFlood();
	}
	
	if( g_debrisModelEnable ){
		this->saveStateDebris();
	}
	
	//////////////////////////////////////////////////////////////////////
	// Ciclo de simulación
	utils::Timer<std::chrono::milliseconds> timerSimAprox;
	std::string lastHeatMapFilePath = "";
	std::vector<std::string> heatMapFiles;
	
	Environment::floodParams_t floodParams = _env->getFloodParams();
	
	timerSimAprox.start();
	std::cout << "Simulando... tick>0" << std::endl;
	for(g_currTimeSim = 1; g_currTimeSim <= _duration; g_currTimeSim++) {

		if(g_showProgressBar) {
			pg.update(g_currTimeSim);
		}

		auto start = std::chrono::high_resolution_clock::now(); //Measure Time
		
		
		/*if(_env->getFloodParams().enable){
			_env->enableFloodLevelUpdate();
		}*/
		
		_env->updateQuads();
		auto endQuads = std::chrono::high_resolution_clock::now();
		_env->updateAgents();
		auto endAgents = std::chrono::high_resolution_clock::now();
		//_env->updateQuads();

		auto end = std::chrono::high_resolution_clock::now(); //Measure Time
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		auto elapsedQuads  = std::chrono::duration_cast<std::chrono::milliseconds>(endQuads - start);
		auto elapsedAgents = std::chrono::duration_cast<std::chrono::milliseconds>(endAgents - endQuads);
		g_timeExecSim += elapsed.count();
		
		if( _numExperiment >= 0){
			g_logStepDelay << _numExperiment << ":";
		}
		g_logStepDelay << g_currTimeSim         << ":";
		g_logStepDelay << elapsedQuads.count()  << ":";
		g_logStepDelay << elapsedAgents.count() << ":";
		g_logStepDelay << elapsed.count()       << ":";
		g_logStepDelay << g_timeExecSim         << "\n";
		
		
		if(_agentsOut && ((g_currTimeSim % _interval) == 0)) {
			this->savePositionAgents();
			//this->saveStatePatchAgents();
		}

		if(_statsOut && ((g_currTimeSim % _statsInterval) == 0)) {
			_env->updateStats();
			this->saveStats();
			
		}
		
		if(_heatMapOut && ((g_currTimeSim % _heatMapInterval) == 0)){
			lastHeatMapFilePath = this->saveStatePatchAgents();
			heatMapFiles.push_back(lastHeatMapFilePath);
		}
		
		if(floodParams.enable  &&((g_currTimeSim % floodParams.sampleStateInterval) == 0)){
			if(floodParams.imagesEnable){
				// crea imagen PGM del estado de la inundación
				this->saveImgFlood();
			}
			
			if(floodParams.stateEnable){
				// crea archivo del estado de la inundación
				this->saveStateFlood();
			}
		}
		
		
		if( (g_currTimeSim % _samplingInterval) == 0 && _heatMapOut){
			this->samplingSim();
			if(g_TSevacAll[g_currTimeSim] >= _samplingLevel && _compareWithOthersSimIsOn){
				
				//std::cout << g_currTimeSim << ":" << g_TSevacAll[g_currTimeSim] << std::endl;
				
				///////////////////////////////////////////////////////////////
				// Se busca simulaciones parecidas a través del servicio rest.
				//								
				json timeSerieGlobal_Qry_json;
				utils::timeSerieToJSON(g_TSevacAll, timeSerieGlobal_Qry_json);
								
				json timeSerieZones_Qry_json(g_sdataEvacTimeByZone);
				json zonesInfo_Qry_json = sdata_zonesInfo;
				
				// formatear los datos de las series de tiempo de cada zona segura y
				// sus respectivos datos geográficos en la siguiente estructura JSON:
				//	{
				// 			...
				//			"Zn" : {
				//				"evacData" : {
				//					...
				//					"timestamp" : %evac
				//					...
				//				},
				//				"geographicData": {
				//					"centroide" : [longitud, latitud],
				//					"requiv"    : [rw, rh]
				//				}
				//			}
				//			...
				//	}
				json dataZones;
				dataZones[utils::key::evacByZone] = timeSerieZones_Qry_json;		
				dataZones[utils::key::zonesInfo ] = zonesInfo_Qry_json;	
				
				json evacByZone_Qry;
				utils::transformRawDataJsonTOtsLocalByZone(dataZones, evacByZone_Qry);
	
				std::string options;
				options  = "timeSerieGlobal=" + timeSerieGlobal_Qry_json.dump();
				options += "&evacByZone=" + evacByZone_Qry.dump();
				options += "&seekradius= " + std::to_string(seekRadius);
				options += "&xsteps= " + std::to_string(xsteps);
				options += "&cutoff= " + std::to_string(cutOff);
				options += "&seekradiusHMap= " + std::to_string(seekRadiusHMap);
				options += "&cutoffHMap= " + std::to_string(cutoffHMap);
				
				std::string heatMapQryFilePath = heatMapFiles[ heatMapFiles.size() - 1 ];
				if( !std::filesystem::exists(heatMapQryFilePath)){
					std::cout  << "Path to heatMap " << heatMapQryFilePath << " no exist." << std::endl;
				}
				options += "&heatMapFile= " + heatMapQryFilePath;
	
				//Timer<std::chrono::milliseconds> timer1;
				//timer1.start();				
				try{
					utils::restClient_get(restURL + "/search?" + options, similarSims);
				} catch(std::exception& e){
					std::cerr << e.what() <<std::endl;
				}
				timerSimAprox.stop();
				similarSims["timeSimAprox"] = timerSimAprox.elapsed();
				similarSims["timeExecSim"] = g_timeExecSim;
				//timer1.stop();				
				
				//_samplingLevel += 0.1;
				_compareWithOthersSimIsOn = false;
				
			}
		} // Fin del proceso de muestreo y comparación con simulaciones anteriores.

	}
	//Fin ciclo de simulación
	
	_execForMTX.lock();
	_simInExec = false;
	_execForMTX.unlock();
	
	watchDogThread.join();
	
	if(g_showProgressBar) {
		std::cout << std::endl;
	}

	this->executionSummary();

	if( _statsOut) {
		this->makeStats();
	}
	
	////////////////////////////////////////////////////////
	//  Crear el archivo JSON de la animación de la simulación
	//if(_agentsOut){
	//}
	json animacionConfig = {
		{"pathFileSim", _fsettings["output"]["agents-path"]},
		{"frameMin", 0},
		{"frameMax", _duration},
		{"frameStep", _interval},
		{"deltaT", g_deltaT},
		{"mapOffset", 0.01}, //Offset del borde mapa (0.01 ==> 800metros )
		{"pathFileDebrisState", _fsettings["debrisParams"]["stateDir"].get<std::string>() + "/" + _fsettings["debrisParams"]["infoFile"].get<std::string>()},
		{"pathFileFloodStateDir", _fsettings["floodParams"]["stateDir"]},
		{"criticaFloodlLevel", _fsettings["floodParams"]["criticalLevel"]},
		{"input",{
			{"zones", "input/"+ _fsettings["input"]["zones"].get<std::string>()}
		}}
	};
	
	std::ofstream ofs( _animConfig );
	ofs << animacionConfig.dump(4) << std::endl;
		
	
	
	/*
	// prueba
	std::cout <<  "initial_zones:" << sdata_zonesInfo["initial_zones"].dump(4) << std::endl;
	for(const auto& z : sdata_zonesInfo["initial_zones"]){
		json centroide = z["centroide"];
		json requiv    = z["requiv"];
		std::cout << "centroide:" << centroide << std::endl;
		std::cout << "requiv   :" << requiv << std::endl;
	}
	
	std::cout << "reference_zones:" << sdata_zonesInfo["reference_zones"].dump(4) << std::endl;
	for(const auto& z : sdata_zonesInfo["reference_zones"]){
		json centroide = z["centroide"];
		json requiv    = z["requiv"];
		std::cout << "centroide:" << centroide << std::endl;
		std::cout << "requiv   :" << requiv << std::endl;
	}
	*/
	
	////////////////////////////////////////////////////////////////////////
	// Convertir los datos de muestreo de la simulación a formato JSON
	json sdata_DistSafeZoneByAgeGroup(g_sdataDistSafeZoneByAgeGroup);
	json sdata_EvacTimeByZone(g_sdataEvacTimeByZone);
	
	////////////////////////////////////////////////////////////////////////
	// Juntar las características de las zonas con el muestreo de la simulación
	// en un sólo JSON
	json samplingDataSim;
	
	samplingDataSim["id"]           = _fsettings["city"];
	samplingDataSim["description"]  = _fsettings["description"].get<std::string>();
	samplingDataSim["uuid"]         = _uuidSim;
	samplingDataSim["totalAgents"]  = g_totalAgentsInSim;
	samplingDataSim["zonesInfo"]    = sdata_zonesInfo;
	samplingDataSim["nonEvacByAge"] = sdata_DistSafeZoneByAgeGroup;
	samplingDataSim["evacByZone"]   = sdata_EvacTimeByZone;
	json evcTime(g_evacTime);
	samplingDataSim["evacTime"]     = evcTime;
	samplingDataSim["heatMapPath"] = _heatMapPath;
	
	/*
	std::cout << "---------------JSON dump---------------------" << std::endl;
	std::cout << "---------------------------------------------" << std::endl;
	std::cout << std::setw(4) << samplingDataSim                 << std::endl;
	std::cout << "---------------------------------------------" << std::endl;
	*/
	
	if(_saveSimInDB){
		RestClient::Response r = RestClient::post(restURL + "/cities", "application/json", samplingDataSim.dump());
	
		if(r.code == 200){
			json response = json::object();
			response = json::parse(r.body);

			std::cout << "Body:" << response.dump(4) << std::endl;
		}
		else{
			if(r.code == COULDNT_CONNECT){
				std::cout << "Could not connect to REST server." << std::endl;
			}
			else{
				std::cout << "Error: "<< r.code << std::endl;
			}
		}
	}
	
	
	
	std::cout << std::endl;
	std::cout << "Sampling level           : " << _samplingLevel << std::endl;
	std::cout << "UUID sim                 : " << _uuidSim << std::endl;
	std::cout << "Simulation descriptiom   : " << _fsettings["description"].get<std::string>() << std::endl;
	std::cout << "Elapsed real sim. time   : " << g_timeExecSim << std::endl;
	std::cout << "Elapsed aprox. sim. time : " << similarSims["timeExecSim"] << std::endl;
	
	//std::cout << similarSims.dump(4) << std::endl;
	
	std::cout << "==== Similar simulations by Time Series ====" << std::endl;
	for(const auto& element : similarSims["TS"]["dataFound"]["simFounds"]){
		std::cout << element["distance"]         << ":";
		std::cout << element["uuid"]             << ":";
		std::cout << element["description"]      << std::endl;
	}
	
	std::cout << "==== Similar simulations by Heat Maps ====" << std::endl;
	for(const auto& element : similarSims["HM"]["dataFound"]["simFounds"]){
		std::cout << element["distance"]         << ":";
		std::cout << element["uuid"]             << ":";
		std::cout << element["description"]      << std::endl;
	}
	
	
	
	
	
	
	
}


Simulator::~Simulator(void)
{
	;
}

////////////////////////////////////////////////////////
//	Simulator::savePositionAgents()
//
void Simulator::savePositionAgents()
{
	std::ostringstream ss;
	ss << std::setw( 10 ) << std::setfill( '0' ) << g_currTimeSim;

	std::string nameFile = ss.str() + "." + _filesimSufix ;
	std::string pathFile = _filesimPath + "/" + nameFile ;


	std::ostringstream ssStateAgents;
	for( auto& agent : _env->getAgents() ) {
		double latitude,longitude,h;
		///////////////////////
		//  Definición:
		//       https://geographiclib.sourceforge.io/html/classGeographicLib_1_1LocalCartesian.html
		_env->getProjector().Reverse(agent->position()[0],agent->position()[1],0,latitude,longitude,h);
		
		// Rapidez del agente
		Vector2D curVel = agent->currVelocity();
		double   curSpeed = sqrt( curVel.squared_length());
		
		Vector2D direction = agent->direction();
		
		ssStateAgents << agent->id() << " ";
		ssStateAgents << std::fixed << std::setprecision(_filesimPrecision) << latitude << " " << longitude;
		ssStateAgents << " " << agent->model();
			//<< " " << agent->getNextTimeUsePhone()
			//<< " " << agent->getProbUsePhone()
			//<< " " << agent->getAgentNeighborsSize()
			//<< " " << agent->getDensity()
		ssStateAgents << " " << agent->getUsingPhone();
		
		ssStateAgents << " " << agent->isAlive();
		
		ssStateAgents << " " << curSpeed;
		
		if(curSpeed > 0){
			ssStateAgents << " [" << curVel/curSpeed << "]";
		}
		else{
			ssStateAgents << " [NA NA]";
		}
		
		ssStateAgents << " [" << direction << "]";
		
		ssStateAgents << std::endl;
	}
	
	std::ofstream ofs(pathFile);
	ofs << ssStateAgents.str();
	
	
}

////////////////////////////////////////////////////////
//	Simulator::saveStatePatchAgents()
//
std::string Simulator::saveStatePatchAgents()
{	
	Environment::grid_t gridInfo = _env->getGrid();

	// Valor maximo para el nivel de gris.
	uint32_t radioGroup = _heatMapSize;
	
	// El valor de maxGrayValue de la imagen PGM también
	// se utiliza para uniformar la máxima cantidad de agentes que hay
	// en un patchGroup entre simulaciones, con el fin de compararlas
	// entre sí.
	// Para 100000 agentes, la máxima cantidad de agentes en un patchGroup
	// es 350 para un patchGroup de 1x1 y 380 para un patchGroup de 2x2.
	uint32_t maxGreyValue = 400; //gridInfo._quadSize * radioGroup;
		
	std::ostringstream ss;
	ss << std::setw( 10 ) << std::setfill( '0' ) << g_currTimeSim;

	std::string nameFile = ss.str() + ".pgm" ;
	std::string pathFile = _heatMapPath + "/" + nameFile ;

	
	/*for( auto& pAgent : _env->getPatchAgents() ) {
		PatchAgent::quad_t quadInfoPatchAgent;
		uint32_t totalAgentsInPatch;
		quadInfoPatchAgent = pAgent->getQuadInfo(); 
		totalAgentsInPatch = pAgent->getAgents().size();
		
		// Coordenada X,Y del patchAgent: quadInfoPatchAgent.idX, quadInfoPatchAgent.idY 
		
		// Para las coordenadas de los pixeles de la imagen, hay que 
		// convertir el eje Y, ya que los patchAgents comienzan en 0,0 en el 
		// vertice inferior izquierdo y la imagen PGM comienza en 0,0 en el
		// vertice superior izquierdo.
		uint32_t imgX = quadInfoPatchAgent.idX;
		uint32_t imgY = (gridInfo._quadY - 1) - quadInfoPatchAgent.idY;
		
		imgTest.value(imgY, imgX, imgTest.maxGreyValue() - totalAgentsInPatch);

	}*/
	
	
	// la imagen PGM se crea por filas x columna.
	// las filas son las coordenadas Y del patchAgent
	// las columnas son las coordenadas X del patchAgent
	
	uint32_t PGMquadX, PGMquadY;
	PGMquadY = gridInfo._quadY / radioGroup;
	PGMquadX = gridInfo._quadX / radioGroup;
	
	//std::cout << "PGMquadX:" << PGMquadX << ", PGMquadY:" << PGMquadY << "\n";
	
	PGM imgTest(PGMquadY, PGMquadX);
	imgTest.maxGreyValue( maxGreyValue );

	//uint32_t maxAgentsInPatchGroup = 0;
	//uint32_t countPatchGroup = 0; 
	//uint32_t totalAgents = 0;
	for(uint32_t xx = 0; xx < PGMquadX; xx += 1){
		for(uint32_t yy = 0; yy < PGMquadY; yy += 1){			
			uint32_t xx_min, yy_min;
			uint32_t xx_max, yy_max;
			
			xx_min = xx * radioGroup;
			yy_min = yy * radioGroup;
			xx_max = xx_min + radioGroup - 1;
			yy_max = yy_min + radioGroup - 1;
			
			xx_max >= gridInfo._quadX - 1 ? xx_max =  gridInfo._quadX - 1 : xx_max = xx_max;
			yy_max >= gridInfo._quadY - 1 ? yy_max =  gridInfo._quadY - 1 : yy_max = yy_max;
			
			
			// Contar los agentes que hay en el grupo de patchAgents
			uint32_t totalAgentsInGroup = 0;
			for(uint32_t Xgroup = xx_min; Xgroup <= xx_max; Xgroup++){
				for(uint32_t Ygroup = yy_min; Ygroup <= yy_max; Ygroup++){
					uint32_t idPatchAgentInGroup = Ygroup * gridInfo._quadX + Xgroup;
					
					PatchAgent* pAgent = _env->getPatchAgent(idPatchAgentInGroup);
					totalAgentsInGroup += pAgent->getAgents().size();
					
				}
			}
			//maxAgentsInPatchGroup = std::max(maxAgentsInPatchGroup, totalAgentsInGroup);
			
			// Sólo se consideran para el promedio los grupos que tienen agentes
			//if(totalAgentsInGroup > 0){
			//	totalAgents += totalAgentsInGroup;
			//	countPatchGroup++;
			//}
			
			// Para las coordenadas de los pixeles de la imagen, hay que 
			// convertir el eje Y, ya que los patchAgents comienzan en 0,0 en el 
			// vertice inferior izquierdo y la imagen PGM comienza en 0,0 en el
			// vertice superior izquierdo.
			uint32_t imgX = xx;
			uint32_t imgY = (PGMquadY - 1) - yy;

			if(totalAgentsInGroup >= imgTest.maxGreyValue()){
				//std::cout << "Overlflow-> totalAgentsInGroup:" << totalAgentsInGroup << std::endl;
				totalAgentsInGroup = imgTest.maxGreyValue();
			}
			
			imgTest.value(imgY, imgX, totalAgentsInGroup);
		}
	}
	imgTest.invert();
	imgTest.toFile(pathFile);
	//std::cout << "maxAgentsInPatchGroup:" << maxAgentsInPatchGroup;
	//std::cout << "\ttotalAgents:" << totalAgents;
	//std::cout << "\tAvg AgentsInPatchGgroup:" << (double)totalAgents / (double)countPatchGroup; 
	//std::cout << std::endl;
	
	return(pathFile);
}

void Simulator::saveStateFlood()
{
	Environment::floodParams_t floodParams = _env->getFloodParams();
	
	std::ostringstream ss;
	ss << std::setw( 10 ) << std::setfill( '0' ) << g_currTimeSim;

	std::string nameFile = ss.str() + ".txt" ;
	std::string pathFile = floodParams.stateDir + "/" + nameFile ;
	
	std::ostringstream ssStateFlood;
	for(auto& floodZone : _env->getFloodZones()) {
		for(auto& idPatchAgent : floodZone.patchAgentsInZone() ){
			PatchAgent* pAgent = _env->getPatchAgent(idPatchAgent);			
			PatchAgent::quad_t qInfo = pAgent->getQuadInfo();
			double lFlood = pAgent->getLevelFlood();
			double maxLevelFlood = pAgent->getMaxLevelFlood();
			
			if(lFlood > 0){
				// Coordenadas xc,yc del centro del cuadrante donde está el patchAgent.
				// qInfo.{x0,y0} son las coordenadas de la esquina inferior izquierda
				double xc = qInfo.x0 + qInfo.width / 2;
				double yc = qInfo.y0 + qInfo.height / 2;
			
				double latitude,longitude,h;
				///////////////////////
				//  Definición:
				//       https://geographiclib.sourceforge.io/html/classGeographicLib_1_1LocalCartesian.html
				_env->getProjector().Reverse(xc, yc, 0, latitude, longitude, h);
			
				ssStateFlood << pAgent->getId() << " ";
				ssStateFlood << std::fixed << std::setprecision(_filesimPrecision) << latitude << " " << longitude << " ";
				ssStateFlood << lFlood;
				ssStateFlood << " " << maxLevelFlood;
				ssStateFlood << std::endl;
			}
			
			
			
		}

	}
	std::ofstream ofs(pathFile);	
	ofs << ssStateFlood.str() << std::endl;
}

void Simulator::saveStateDebris()
{
	std::ostringstream ssStateDebris;
	
	for(const auto& pAgentInCity : _env->getPatchAgentsInCity() ){
		if(!pAgentInCity->isDebrisFree()){
			// El patch agent tiene datos de escombros
			PatchAgent::quad_t qInfo = pAgentInCity->getQuadInfo();
			double probDebris = pAgentInCity->getProbDebris();
			
			
			// Coordenadas xc,yc del centro del cuadrante donde está el patchAgent.
			// qInfo.{x0,y0} son las coordenadas de la esquina inferior izquierda
			double xc = qInfo.x0 + qInfo.width / 2;
			double yc = qInfo.y0 + qInfo.height / 2;
		
			double latitude,longitude,h;
			///////////////////////
			//  Definición:
			//       https://geographiclib.sourceforge.io/html/classGeographicLib_1_1LocalCartesian.html
			_env->getProjector().Reverse(xc, yc, 0, latitude, longitude, h);
		
			ssStateDebris << pAgentInCity->getId() << " ";
			ssStateDebris << std::fixed << std::setprecision(_filesimPrecision) << latitude << " " << longitude << " ";
			ssStateDebris << probDebris;
			ssStateDebris << std::endl;
								
			
			
		}

	}

	std::string nameFile = _fsettings["debrisParams"]["infoFile"].get<std::string>();
	std::string pathFile = _debrisFilePath + "/" + nameFile ;
	
	std::ofstream ofs(pathFile);	
	ofs << ssStateDebris.str() << std::endl;
	
}

void Simulator::saveImgFlood()
{	
	Environment::floodParams_t floodParams = _env->getFloodParams();
	Environment::grid_t gridInfo = _env->getGrid();

	// Cantidad de patchs por pixel
	uint32_t radioGroup = _heatMapSize;
	
	// El valor de maxGrayValue de la imagen PGM 
	// se utiliza para uniformar el nivel del agua en un patchGroup.
	// Representa la altura maxima _env->getFloodParams().criticalLevel
	uint32_t maxGreyValue = 255; 
	
	// Factor de conversión greyLevel/levelFlood
	double convFactor =  (double)255.0 / floodParams.criticalLevel;
	
		
	std::ostringstream ss;
	ss << std::setw( 10 ) << std::setfill( '0' ) << g_currTimeSim;

	std::string nameFile = ss.str() + "-flood.pgm" ;
	std::string pathFile = floodParams.imagesDir + "/" + nameFile ;

	
	
	// la imagen PGM se crea por filas x columna.
	// las filas son las coordenadas Y del patchAgent
	// las columnas son las coordenadas X del patchAgent
	
	uint32_t PGMquadX, PGMquadY;
	PGMquadY = gridInfo._quadY / radioGroup;
	PGMquadX = gridInfo._quadX / radioGroup;
	
	//std::cout << "PGMquadX:" << PGMquadX << ", PGMquadY:" << PGMquadY << "\n";
	
	PGM imgTest(PGMquadY, PGMquadX);
	imgTest.maxGreyValue( maxGreyValue );

	//uint32_t maxAgentsInPatchGroup = 0;
	//uint32_t countPatchGroup = 0; 
	//uint32_t totalAgents = 0;
	for(uint32_t xx = 0; xx < PGMquadX; xx += 1){
		for(uint32_t yy = 0; yy < PGMquadY; yy += 1){			
			uint32_t xx_min, yy_min;
			uint32_t xx_max, yy_max;
			
			xx_min = xx * radioGroup;
			yy_min = yy * radioGroup;
			xx_max = xx_min + radioGroup - 1;
			yy_max = yy_min + radioGroup - 1;
			
			xx_max >= gridInfo._quadX - 1 ? xx_max =  gridInfo._quadX - 1 : xx_max = xx_max;
			yy_max >= gridInfo._quadY - 1 ? yy_max =  gridInfo._quadY - 1 : yy_max = yy_max;
			
			
			// determiner el nivel de inundación promedio en el grupo de patchAgents 'avgLevelFlood'
			double avgLevelFlood = 0;
			for(uint32_t Xgroup = xx_min; Xgroup <= xx_max; Xgroup++){
				for(uint32_t Ygroup = yy_min; Ygroup <= yy_max; Ygroup++){
					uint32_t idPatchAgentInGroup = Ygroup * gridInfo._quadX + Xgroup;
					
					PatchAgent* pAgent = _env->getPatchAgent(idPatchAgentInGroup);
					avgLevelFlood += pAgent->getLevelFlood();
					/*if(pAgent->isFloodable()){
						std::cout << "avgLevelFlood: " << avgLevelFlood << std::endl;
						std::cout << "pAgent->getLevelFlood(): " << pAgent->getLevelFlood() << std::endl;
					}*/
				}
			}
			avgLevelFlood /= radioGroup*radioGroup;
			
			
			// El nivel de agua promedio en el patchGroup se convierte 
			// a nivel del gris para la imagen
			uint32_t greyLevel = std::round(convFactor * avgLevelFlood);
			//std::cout << (double)avgLevelFlood << std::endl;
			//std::cout << greyLevel << std::endl;
			
			// Para las coordenadas de los pixeles de la imagen, hay que 
			// convertir el eje Y, ya que los patchAgents comienzan en 0,0 en el 
			// vertice inferior izquierdo y la imagen PGM comienza en 0,0 en el
			// vertice superior izquierdo.
			uint32_t imgX = xx;
			uint32_t imgY = (PGMquadY - 1) - yy;

			if(greyLevel >= imgTest.maxGreyValue()){
				//std::cout << "Overlflow-> totalAgentsInGroup:" << totalAgentsInGroup << std::endl;
				greyLevel = imgTest.maxGreyValue();
			}
			
			imgTest.value(imgY, imgX, greyLevel);
		}
	}
	imgTest.invert();
	imgTest.toFile(pathFile);
	//std::cout << "maxAgentsInPatchGroup:" << maxAgentsInPatchGroup;
	//std::cout << "\ttotalAgents:" << totalAgents;
	//std::cout << "\tAvg AgentsInPatchGgroup:" << (double)totalAgents / (double)countPatchGroup; 
	//std::cout << std::endl;
}


////////////////////////////////////////////////////////
//	Simulator::saveStats()
//
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

////////////////////////////////////////////////////////
//	Simulator::executionSummary()
//
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

////////////////////////////////////////////////////////
//	Simulator::makeStats()
//
void Simulator::makeStats()
{
	std::string pathFile01;
	std::string pathFile02;
	
	std::ofstream ofs01;
	
	std::string nameSuffix = this->getNameSuffix();
	
	////////////////////////////////////////////////////////
	//	zonesDensity
	//
	pathFile01 = _statsPath + "/zonesDensity" +  nameSuffix;
	ofs01.open(pathFile01);
	for(auto& item : g_logZonesDensity) {
		
		if( _numExperiment >= 0){
			ofs01 << _numExperiment << ":";
		}
		
		ofs01 << item << std::endl;
	}
	ofs01.close();
	
	////////////////////////////////////////////////////////
	//	usePhone
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
	
	
	////////////////////////////////////////////////////////
	//	Deceased agents
	pathFile01 = _statsPath + "/deceasedAgents" + nameSuffix ;
	ofs01.open(pathFile01);
	
	if( _numExperiment >= 0){
		ofs01 << "numExperiment:" ;
	}
		
	ofs01 << "timeStamp:nInSafeZonesAgents:nMovingAgents:nDeceasedAgents:nWaitingAgents:";
	ofs01 << "pInSafeZonesAgents:pMovingAgents:pDeceasedAgents:pWaitingAgents" << std::endl;
	for(auto& item : g_logDeceasedAgents) {
		if( _numExperiment >= 0){
			ofs01 << _numExperiment << ":";
		}
		
		ofs01 << item << std::endl;
	}
	
	ofs01.close();
		
	
	////////////////////////////////////////////////////////
	//	SIRpanic
	pathFile01 = _statsPath + "/SIRpanic" + nameSuffix ;
	ofs01.open(pathFile01);
	
	if( _numExperiment >= 0){
		ofs01 << "numExperiment:" ;
	}
		
	ofs01 << "timeStamp:nS:nI:nR:S:I:R" << std::endl;		
	
	
	for(auto& item : g_logSIRpanic) {

		if( _numExperiment >= 0){
			ofs01 << _numExperiment << ":";
		}
		
		ofs01 << item << std::endl;
	}
	
	ofs01.close();
	
	////////////////////////////////////////////////////////
	//	velocity
	//
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
	
	////////////////////////////////////////////////////////
	//	summary
	//
	pathFile01 = _statsPath + "/summary" + nameSuffix ;
	ofs01.open(pathFile01);
	
	if( _numExperiment >= 0){
		ofs01 << "numExperiment:" ;
	}
	
	std::random_device _randomDevice;
	std::uniform_real_distribution<> unifDistro(-(float)_interval*g_deltaT/2.0, (float)_interval*g_deltaT/2.0); //2020-10-09

	ofs01 << "id:model:groupAge:safeZone:distanceToTargetPos:responseTime:evacTime:initialZone:travelDistance:isAlive:timeLived" << std::endl;		
	for(auto& fooAgent : _env->getAgents()){
		if( _numExperiment >= 0){
			ofs01 << _numExperiment << ":";
		}
		
		//aleatorizar el tiempo de evacuación para que esté 
		//en un rango tevac +/- (_interval)/2
		double evacTime = fooAgent->getEvacuationDataStruct().evacuationTime;
		if( evacTime >  (float)_interval*g_deltaT/2.0 ){ //2020-10-09
			evacTime +=  unifDistro(_randomDevice) ; 
		}
		
		//guardar los tiempos de evacuacion para 
		//enviarlos al back-end.
		g_evacTime.push_back(evacTime);
		
		ofs01 << std::to_string(fooAgent->id()) << ":"  
			<< fooAgent->model() << ":" 
			<< std::to_string(fooAgent->groupAge()) << ":" 
			<< fooAgent->getSafeZoneID() << ":" 
			<< std::to_string(fooAgent->distanceToTargetPos()) << ":" 
			<< std::to_string(fooAgent->responseTime()) << ":" 
			<< std::to_string(evacTime) << ":"
			//	<< " <" << std::to_string(fooAgent->inSafeZone()) << "> :" //
			<< fooAgent->getInitialZoneID() << ":"
			<< std::to_string(fooAgent->getEvacuationDataStruct().travelDistance) << ":"
			<< std::to_string(fooAgent->getEvacuationDataStruct().isAlive) << ":"
			<< std::to_string(fooAgent->getEvacuationDataStruct().timeLived)
			<< std::endl;	
	}
	ofs01.close();
	
	////////////////////////////////////////////////////////
	//	step delay
	//
	pathFile01 = _statsPath + "/stepDelay" + nameSuffix ;
	ofs01.open(pathFile01);
	
	if( _numExperiment >= 0){
		ofs01 << "numExperiment:" ;
	}
	
	ofs01 << "time:DeltaTimeQuads:DeltaTimeAgents:DeltaTimeExecSim:timeExecSim\n";		
	ofs01 << g_logStepDelay.str();
	
	ofs01.close();
}

////////////////////////////////////////////////////////
//	Simulator::samplingSim()
//
void Simulator::samplingSim()
{

	////////////////////////////////////////////////////////////////////
	// Determinar los tiempos de evacuación agrupados por zona segura.
	// Se almacenan en el map evacTimeByZone
	std::map<std::string, std::vector<double> > evacTimeByZone;
	std::map<std::string, std::vector<double> > distancesByGroupAge;
	
	for(auto& referenceZone : _env->getReferenceZones() ) {	
		std::vector<double> evacTimeList;
		
		//Por cada Zona Segura, consultar por los agentes aque están asignados a ella
		std::set<uint32_t> idsAgentsAssigned = referenceZone.agentsAssignedInZone();		
		for(auto& idAg : idsAgentsAssigned){
			auto fooAg = _env->getAgent(idAg);			

			//Si el agente no esta evacuado, entonces
			//     agregar su distancia a su zona de seguridad al map 'distancesByGroupAge'
			//En caso contrario
			//     agregar su tiempo de evacuación a la lista 'evacTimeList'
			if(!fooAg->inSafeZone()){	
				std::string labelGroupAge = "G" + std::to_string(fooAg->groupAge());	
				distancesByGroupAge[labelGroupAge].push_back(fooAg->distanceToTargetPos());
			}
			else{
				evacTimeList.push_back(fooAg->getEvacuationDataStruct().evacuationTime );
			}			
		}	
		
		std::string idZone = referenceZone.getNameID();		
		evacTimeByZone[idZone] = evacTimeList;
	}
	
	/*
	////////////////////////////////////////////////////////////////////
	// Determinar la cantidad total de no-evac
	uint32_t nonEvacTotal = 0;
	for (const auto& [ageGroup, distancesList] : distancesByGroupAge) {
		uint32_t nonEvacNumber = distancesList.size();	
		nonEvacTotal += nonEvacNumber;
	}
	*/
	
	////////////////////////////////////////////////////////////////////
	// Determinar las métricas de distancia a los zonas seguras de los
	// agentes no-evacuados, agrupados por grupo etário, para el tiempo 
	// de simulación actual 'g_currTimeSim'
	// Por cada grupo etário, se incluye la siguiente tupla:
	//      cantidad de no-evacuados
	//      porcentaje de no-evacuados sobre el total de la simulación
	//      distancia mínima
	//      distancia máxima
	//      distancia promedio
	// Se almacenan en el map 'g_sdataDistSafeZoneByAgeGroup'
	for (const auto& [ageGroup, distancesList] : distancesByGroupAge) {
		uint32_t nonEvacNumber = distancesList.size();
		
		float minDist, maxDist, meanDist;
		minDist=maxDist=meanDist=0;
		
		if(nonEvacNumber > 0){
			auto [minDistG, maxDistG] = std::minmax_element(begin(distancesList), end(distancesList));
			minDist = *minDistG;
			maxDist = *maxDistG;
			
			meanDist = (float)std::accumulate(distancesList.begin(), distancesList.end(), 0) / nonEvacNumber;
		}
		
		float pNonEvac;
		//pNonEvac = (float)nonEvacNumber / (float)nonEvacTotal;
		pNonEvac = (float)nonEvacNumber / (float)g_totalAgentsInSim;

		g_sdataDistSafeZoneByAgeGroup[std::to_string(g_currTimeSim)][ageGroup] = std::make_tuple(nonEvacNumber, pNonEvac, minDist, maxDist, meanDist);
	}
	
	/*
	////////////////////////////////////////////////////////////////////
	// Determinar la cantidad de agentes asignados por cada zona segura.
	std::map<std::string, uint32_t> agentsAssignedInZone;
	for(auto& referenceZone : _env->getReferenceZones() ) {	
		agentsAssignedInZone[referenceZone.getNameID()] = referenceZone.getTotalAgentsAssigned();
	}
	*/
	
	////////////////////////////////////////////////////////////////////
	// Determinar los tiempos de evacuación por cada zona segura para
	// el tiempo de simulación actual 'g_currTimeSim'
	// Por cada zona segura, se incluye la siguiente tupla:
	//     cantidad de evacuados
	//     porcentaje de evacuados sobre el total de agentes de la simulación
	//     tiempo de evacuación mínimo
	//     tiempo de evacuación máximo
	//     tiempo de evacuación promedio 
	// Se almacenan en el map 'g_sdataEvacTimeByZone'
	uint32_t evacAgents = 0;
	for (const auto& [zoneID, evacTimeList] : evacTimeByZone) {
		uint32_t evacNumber = evacTimeList.size();
	
		evacAgents += evacNumber;
		
		float minEvacTime, maxEvacTime, meanEvacTime;
		minEvacTime=maxEvacTime=meanEvacTime=0;
	
		if(evacNumber > 0){
			auto [minEvacTimeG, maxEvacTimeG] = std::minmax_element(begin(evacTimeList), end(evacTimeList));
			minEvacTime = *minEvacTimeG;
			maxEvacTime = *maxEvacTimeG;
		
			meanEvacTime = (float)std::accumulate(evacTimeList.begin(), evacTimeList.end(), 0) / evacNumber;
		}
			
		float pEvac;	
		//uint32_t totalAgentsAssignedInZone = agentsAssignedInZone[zoneID];
		//pEvac = (float)evacNumber / (float)totalAgentsAssignedInZone;
		pEvac = (float)evacNumber / (float)g_totalAgentsInSim;
		
		g_sdataEvacTimeByZone[std::to_string(g_currTimeSim)][zoneID] = std::make_tuple(evacNumber, pEvac, minEvacTime, maxEvacTime, meanEvacTime);
	}
	
	
	// Crear las TS para que la simulación en curso se puede comparar con 
	// las simulaciones almacenadas.
	g_TSevacAll[g_currTimeSim] = evacAgents / (double)g_totalAgentsInSim;

}

////////////////////////////////////////////////////////
//	Simulator::getNameSuffix()
//
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

////////////////////////////////////////////////////////
//	método que vigila el avance de la simulación cada 'deltaTime'.
//  Si no avanza por 'thresTime' segundos, elimina la simulación actual.
//
void Simulator::watchDog(uint32_t initialWaitTime,  uint32_t deltaTime,  uint32_t thresTime, std::string dirTodDelete)
{
	std::this_thread::sleep_for(std::chrono::seconds(initialWaitTime));
	
	uint32_t tickOld = 0;
	uint32_t tickNew = 0;
	
	uint32_t timesBlocked = 0;
    while (1){
		tickNew = g_currTimeSim;
		
		_execForMTX.lock();
		if(!_simInExec){
			std::cout << "ciclo terminado\n";
			break;
		}
		_execForMTX.unlock();
		
		if(tickNew == tickOld){
			uint32_t blockedTime;
			
			timesBlocked++;
			blockedTime = timesBlocked*deltaTime;
			if(timesBlocked*deltaTime >= thresTime){
				std::cout << "El ciclo no ha avanzado por " << blockedTime << " s. Forzando salida.\n";
				
				if (std::filesystem::exists(dirTodDelete) && std::filesystem::is_directory(dirTodDelete)) {
					try {
						std::filesystem::remove(dirTodDelete);
						std::cout << "Deleted " << dirTodDelete << " directory.\n";
					} catch (const std::filesystem::filesystem_error& e) {
						std::cerr << "Error: " << e.what() << "\n";
					}
				} 
				std::raise(SIGTERM); 
			}
			
		}
		else{
			tickOld = tickNew;
			timesBlocked = 0;
		}
		
		std::this_thread::sleep_for(std::chrono::seconds(deltaTime));
    }
}
















