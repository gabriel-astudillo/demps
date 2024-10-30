#include <simulator.hh>




std::mutex Simulator::_execForMTX;
bool Simulator::_simInExec;


Simulator::Simulator(void)
{

}

////////////////////////////////////////////////////////
//	Simulator::simulator()
//
Simulator::Simulator(const json &fsettings, const json& fzones, const std::string &map_osrm)
{
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());

	std::vector<Agent> _agents;

	_fsettings = fsettings;
	
	//Crear el identificador único de la simulación
	_uuidSim = utils::get_uuid();
	
	// Asignación de variables globales del proyecto
	global::execOptions.showProgressBar   = _fsettings["output"]["progressBar"].get<bool>();
	global::params.modelsEnable.panic     = _fsettings["panicModelEnable"].get<bool>(); 
	global::params.modelsEnable.elevation = _fsettings["elevationModelEnable"].get<bool>();
	global::params.modelsEnable.debris    = _fsettings["debrisModelEnable"].get<bool>();
	global::params.modelsEnable.flood     = _fsettings["floodModelEnable"].get<bool>();
	global::params.closeEnough            = _fsettings["closeEnough"].get<float>();
	global::params.randomWalkwayRadius    = _fsettings["randomWalkwayRadius"].get<float>();
	global::params.attractionRadius       = _fsettings["attractionRadius"].get<float>();
	global::params.deltaT                 = _fsettings["deltaT"].get<float>();
	
	_numExperiment = _fsettings["numExperiment"].get<int32_t>();

	_duration        = (uint32_t)(_fsettings["duration"].get<uint32_t>() / global::params.deltaT);
	_calibrationTime = (uint32_t)(_fsettings["calibration"].get<uint32_t>() / global::params.deltaT);
	
	//////////////////////////////////////////////////////////
	// Directorio de salida
	// 
	std::string outputBaseDir = _fsettings["output"]["directory"].get<std::string>() + "/";
	
	*global::serverLog  << "####### Deleting previous results #######\n";
	*global::serverLog  << "####### " <<  outputBaseDir << std::endl;
	std::filesystem::remove_all(outputBaseDir);

	_pidFilePath = outputBaseDir + global::params.watchDog.pidFile;


	global::execOptions.agentsOut = _fsettings["output"]["agents-out"].get<bool>();
	
	_interval         = (uint32_t)(_fsettings["output"]["interval"].get<uint32_t>() /  global::params.deltaT);
	_filesimPrecision = _fsettings["output"]["agents-precision"].get<uint32_t>();
	_filesimSufix     = _fsettings["output"]["agents-sufix"].get<std::string>();

	//////////////////////////////////////////////////////////
	// Directorio donde se almacenan el estado de los agentes
	//
	if(_fsettings["output"]["agents-path"].get<std::string>()[0] != '/'){ // Si es ruta relativa
		_agentsPath = outputBaseDir + _fsettings["output"]["agents-path"].get<std::string>();	
	}
	else{ // Si es ruta absoluta
		_agentsPath = _fsettings["output"]["agents-path"].get<std::string>();
	}

	if( !std::filesystem::exists(_agentsPath)){
		*global::serverLog  << "Path to agents directory:\n\t" << _agentsPath << "\n\tno exist. Creating." << std::endl;
		std::filesystem::create_directories(_agentsPath);
	}
	
	//////////////////////////////////////////////////////////
	// Directorio donde se almacenan los mapas de calor
	//
    _heatMapOut      = _fsettings["output"]["heatMap-out"].get<bool>();
	_heatMapSize     = _fsettings["output"]["heatMap-size"].get<uint32_t>();
	_heatMapInterval = (uint32_t)(_fsettings["output"]["heatMap-interval"].get<uint32_t>() /  global::params.deltaT);
	
	if(_fsettings["output"]["heatMap-path"].get<std::string>()[0] != '/'){ // Si es ruta relativa
		_heatMapPath = outputBaseDir + _fsettings["output"]["heatMap-path"].get<std::string>();
	}
	else{ // Si es ruta absoluta
		_heatMapPath = _fsettings["output"]["heatMap-path"].get<std::string>();
	}

	if( !std::filesystem::exists(_heatMapPath)){ 
		*global::serverLog  << "Path to heatMap directory:\n\t" << _heatMapPath << "\n\tno exist. Creating." << std::endl;
		std::filesystem::create_directories(_heatMapPath);
	}

	//////////////////////////////////////////////////////////
	// Directorio donde se almacena el estado de los escombros
	//
	if(_fsettings["debrisParams"]["stateDir"].get<std::string>()[0] != '/'){ // Si es ruta relativa
		_debrisFilePath = outputBaseDir + _fsettings["debrisParams"]["stateDir"].get<std::string>();
	}
	else{ // Si es ruta absoluta
		_debrisFilePath = _fsettings["debrisParams"]["stateDir"].get<std::string>();
	}

	if( !std::filesystem::exists( _debrisFilePath )){
		*global::serverLog  << "Path to debris directory:\n\t " << _debrisFilePath << "\n\tno exist. Creating." << std::endl;
		std::filesystem::create_directories( _debrisFilePath );
	}
	
	
	_statsOut        = _fsettings["output"]["stats-out"].get<bool>();
	_statsInterval   = (uint32_t)(_fsettings["output"]["stats-interval"].get<uint32_t>() /  global::params.deltaT);

	//////////////////////////////////////////////////////////
	// Directorio donde se almacena los datos que se utilizarán
	// para generar las estadisticas de la simulación
	//
	if(_fsettings["output"]["stats-path"].get<std::string>()[0] != '/'){ // Si es ruta relativa
		_statsPath = outputBaseDir +  _fsettings["output"]["stats-path"].get<std::string>();
	}
	else{ // Si es ruta absoluta
		_statsPath = _fsettings["output"]["stats-path"].get<std::string>();
	}

	if( !std::filesystem::exists( _statsPath )){
		*global::serverLog  << "Path to stats directory:\n\t" << _statsPath << "\n\tno exist. Creating." << std::endl;
		std::filesystem::create_directories( _statsPath );
	}

	global::params.sampling.interval = _fsettings["samplingInterval"].get<uint32_t>();

	
	//////////////////////////////////////////////////////////
	// Directorio donde se almacena la animación
	//
	_animPath = outputBaseDir + _fsettings["output"]["anim-path"].get<std::string>();
	if( !std::filesystem::exists( _animPath )){
		*global::serverLog  << "Path to animation directory:\n\t" << _animPath << "\n\tno exist. Creating." << std::endl;
		std::filesystem::create_directories( _animPath + "/input" );
	}

	std::string zoneFileName = std::filesystem::path(_fsettings["input"]["zones"].get<std::string>()).filename();

	std::filesystem::copy(_fsettings["input"]["zones"].get<std::string>(), _animPath + "/input/" + zoneFileName );
	std::filesystem::copy(global::params.animationDir + global::params.animationFile, _animPath + global::params.animationFile);


	_animConfig = _animPath + _fsettings["output"]["anim-config"].get<std::string>();


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// La siguiente configuración se utilizará hasta
	// que se rediseñe la comparación con simulaciones
	// anteriores.
	global::params.sampling.saveSimInDB           = false;
	global::params.sampling.compareWithOthersSims = false;
	
	/*
	// Todo lo siguiente se debe rediseñar
	// Si _heatMapOut == false, entonces global::params.sampling.compareWithOthersSims = false
	// No se pueden comparar simulaciones sin mapas de calor

	if(_fsettings["output"]["heatMap-path"].get<std::string>()[0] != '/'){
		_heatMapPath = outputBaseDir + _fsettings["output"]["heatMap-path"].get<std::string>();
		*global::serverLog << "Production Simulation.\n";
		*global::serverLog << "\tThe simulation sample will be compared with others.\n";
		*global::serverLog << "\tThe simulation will not be saved in the SnitchServer." << std::endl;
		global::params.sampling.saveSimInDB = false;
		global::params.sampling.compareWithOthersSims = true;
	}
	else{
		// La ruta al directorio de heatMaps es absoluta. 
		//  ==> Los heatMaps se guardan en ese directorio para que futuras 
		//      simulaciones busquen heatMaps parecidos.
		//  ==> Los heatMaps de cada simulacion se diferencian por su uuid
		_heatMapPath = _fsettings["output"]["heatMap-path"].get<std::string>() + "/" + _uuidSim;
		*global::serverLog << "Training Simulation.\n";
		*global::serverLog << "\tThe simulation sample will not be compared with others.\n";
		*global::serverLog << "\tThe simulation sample will be sent to SnitchServer." << std::endl;
			
		global::params.sampling.saveSimInDB = true;
		global::params.sampling.compareWithOthersSims = false;
	}
	*/
	/*
	// todos los mapas de calor se almacenan para realizar simulaciones aproximadas
	//Los heatMaps de cada simulacion se diferencian por su uuid
	_heatMapPath = .... _fsettings["output"]["heatMap-path"].get<std::string>() + "/" + _uuidSim;;
	global::params.sampling.saveSimInDB = true;
	global::params.sampling.compareWithOthersSims = true;
	*global::serverLog  << "Training and production Simulation.\n";
	*global::serverLog  << "\tThe simulation sample will be sent to SnitchServer.\n";
	*global::serverLog  << "\tThe simulation sample will be compared with others." << std::endl;
	*/
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	

	//Tamaño del cuadrante
	uint32_t quadSize = _fsettings["quadSize"].get<uint32_t>(); //quadSize[m] x quadSize[m]

	//Se crea el ambiente vacío.
	_env = std::make_shared<Environment>();
	
	//
	// inicializar la variable estática _myEnv
	// en todas las clases
	Agent::_myEnv      = _env;
	PatchAgent::_myEnv = _env; 
	Zone::_myEnv       = _env;
	Router::_myEnv     = _env;

	_env->setProjector(fzones);
	_env->setRouter(map_osrm);
	
	_env->setGrid(fzones, global::params.offsetMap, quadSize);
	_env->showGrid();
	
	// Carga parámetros del modelo de densidad
	json densityParams = _fsettings["densityParams"];
	densityParams["enable"] = _fsettings["densityModelEnable"];
	_env->setDensityParams( densityParams );
	
	///////////////////////////////////////////////////////////////////////////
	//
	// Modelo de inundación
	//

	// Los directorios de imagenes y estado de la inundación se convierten a rutas absolutas si son relativas
	json floodParams;
	floodParams = _fsettings["floodParams"];

	if(floodParams["imagesDir"].get<std::string>()[0] != '/'){
		floodParams["imagesDir"] = outputBaseDir + floodParams["imagesDir"].get<std::string>();
		if( !std::filesystem::exists(floodParams["imagesDir"].get<std::string>() )){
			std::filesystem::create_directories(floodParams["imagesDir"].get<std::string>());
		}
	}

	if(floodParams["stateDir"].get<std::string>()[0] != '/'){
		floodParams["stateDir"] = outputBaseDir + floodParams["stateDir"].get<std::string>();
		if( !std::filesystem::exists(floodParams["stateDir"].get<std::string>() )){
			std::filesystem::create_directories(floodParams["stateDir"].get<std::string>());
		}

	}
	
	floodParams["enable"] = _fsettings["floodModelEnable"];
	
	_env->setFloodParams( floodParams );
	Environment::floodParams_t simFloodParams = _env->getFloodParams();
	*global::serverLog << "floodParams:\n" ; 
	*global::serverLog << "\tenable               : " << simFloodParams.enable << "\n";
	*global::serverLog << std::setprecision(2);
	*global::serverLog << "\tarrivalTime     (s)  : " << simFloodParams.arrivalTime << "\n"; 
	*global::serverLog << "\tspeedWaterLevel (m/s): " << simFloodParams.speedWaterLevel << "\n"; 
	*global::serverLog << "\tspeedWaterProp  (m/s): " << simFloodParams.speedWaterProp << "\n"; 
	*global::serverLog << "\tcriticalLevel   (m)  : " << simFloodParams.criticalLevel << "\n"; 
	*global::serverLog << "\tminSpeedFactor  [0,1]: " << simFloodParams.minSpeedFactor << std::endl;
	
	
	///////////////////////////////////////////////////////////////////////////
	//
	// Crear patchs agents
	//
	*global::serverLog  << "Creando patch agentes..." << std::endl; 	
	Environment::grid_t gridData = _env->getGrid();

	//#pragma omp parallel for
	for(size_t y = 0; y < gridData._quadY; y++) {
		for(size_t x = 0; x < gridData._quadX; x++) {
			uint32_t idPatch = x + y * gridData._quadX;
			//std::cout << idPatch << std::endl;
			_env->addPatchAgent( new PatchAgent(idPatch) );
			
		}
	}

	
	////////////////////////////////////////////////////////////////
	// Procesar el archivos de zones y cargar:
	//      a) zonas iniciales y zonas seguras
	//      b) zonas inundables
	//      c) zonas geográficas para monitorear flujo de personas
	//          c.1) zoneType == "lineMonitor"
	//          c.2) zoneType == "pointMonitor"
	//
	*global::serverLog  << "Revisando mapa:\n";
	for(const auto& feature : fzones["features"]) {
		std::string zoneType = feature["properties"]["zoneType"].get<std::string>();
		std::string nameID   = feature["properties"]["nameID"].get<std::string>();
		//std::cout << "zoneType:" << zoneType << std::endl;
		if(zoneType == "initial"){
			*global::serverLog  << "\tCreando zona inicial: " << nameID << "\n"; 
			_env->addInitialZone(feature);
		}
		else if(zoneType == "safe"){
			*global::serverLog  << "\tCreando zona segura: " << nameID << "\n"; 
			_env->addReferenceZone(feature);
		}
		else if(zoneType == "flood" && _env->getFloodParams().enable){
			*global::serverLog  << "\tCreando zona inundable: " << nameID << "\n"; 
			_env->addFloodZone(feature);
		}
		else if(zoneType == "lineMonitor"){
			*global::serverLog  << "\tAsignando patch agentes monitores en línea geográfica: " << nameID << "\n"; 
			_env->addLineMonitorZone(feature);
		}
		else if(zoneType == "\tpointMonitor"){
			*global::serverLog  << "Asignando patch agentes monitores en punto geográfico: " << nameID << "\n"; 
			_env->addPointMonitorZone(feature);
		}
	}
	*global::serverLog  << std::endl;
	
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
		//*global::serverLog << "\x1B[0;90m";
		*global::serverLog << "Configurando zonas inundables:" << "\n";
		*global::serverLog << "\tasignando patch agents." << "\n";
		_env->assignPatchAgentsToFloodZones();
		*global::serverLog << "\tdeterminando coordenadas patch agents perimetrales.\n";
		_env->setNSWEPatchAgentsAllZones();
		
		for(auto& floodZone : _env->getFloodZones()) {
			*global::serverLog << "Zona inundable: " << floodZone.getNameID() << "\n";
			*global::serverLog << "\tarea:" << floodZone.getArea() << "\n";
			*global::serverLog << "\ttotal Patchs: " << floodZone.patchAgentsInZone().size() << "\n";
			*global::serverLog << "\tmax level flood (m): " << floodZone.getMaxLevelFlood() << "\n";
		}
		
		*global::serverLog  << "Identificadores idX,idY de patchAgent de referencia: " << "\n";
		Zone::NSWEPatchAgentsAllZone_t idsPatchAgents = _env->getNSWEPatchAgentsAllZones();
		*global::serverLog << "\tmost further North(idY max) : " << idsPatchAgents[0] << "\n";
		*global::serverLog << "\tmost further South (idY min): " << idsPatchAgents[1] << "\n";
		*global::serverLog << "\tmost further West (idX min) : " << idsPatchAgents[2] << "\n";
		*global::serverLog << "\tmost further East (idX max) : " << idsPatchAgents[3] << "\n";
		
		*global::serverLog << std::endl;
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

	
	*global::serverLog  << "Creando agentes..." << std::endl;
	global::simOutputs.agentsMem = this->getMaxMemory();

	auto start = std::chrono::system_clock::now(); //Measure Time

	uint32_t id = 0;
	ProgressBar pg;
	for(auto& fagent : _fsettings["agents"]) {
		uint32_t totalAgents = uint32_t(fagent["number"]);
		pg.start(totalAgents);

		for(uint32_t i = 0; i < totalAgents; i++) {
			if(global::execOptions.showProgressBar) {
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
			//if(i==10){_env->getAgent(i)->showPanic();}
			
		}
	}
	global::simOutputs.totalAgentsInSim = id;
	auto end = std::chrono::system_clock::now(); //Measure Time
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	global::simOutputs.timeExec.makeAgents += elapsed.count();

	global::simOutputs.agentsMem = this->getMaxMemory() - global::simOutputs.agentsMem;

	if(global::execOptions.showProgressBar) {
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
	*global::serverLog  << "Ajustando posición inicial de los agentes..." << std::endl;

	auto start = std::chrono::system_clock::now(); //Measure Time
	_env->adjustAgentsInitialPosition(_calibrationTime);
	_env->updateQuads();

	if(global::execOptions.showProgressBar) {
		std::cout << std::endl;
	}
	
	//
	// Determinar los patch agents que están en la calles donde
	// transitan los agentes
	//
	*global::serverLog  << "Determinando patch agents que contienen calles... " <<  std::endl;
	_env->determinatePAgentsInStreets();
	
	if(global::execOptions.showProgressBar) {
		std::cout << std::endl;
	}
	
	//
	// Determinar proporcion de los patchs agentes que están en las
	// calles que deben tener escombros;
	//
	if(global::params.modelsEnable.debris){
		int pAgentsWithDebris;
		double debrisRatio = _fsettings["debrisParams"]["debrisRatio"].get<double>()/100.0;
		*global::serverLog  << "Determinando patch agents que contienen escombros...\n";
		_env->determinatePAgentsWithDebris(debrisRatio, pAgentsWithDebris);

		*global::serverLog  << "Patch Agents in streets : "<<  _env->getPatchAgentsInStreets().size() << "\n";
		*global::serverLog  << "Patch Agents in streets with debris : "<<  pAgentsWithDebris << " ";
		*global::serverLog  << "("<<  (double)pAgentsWithDebris /  _env->getPatchAgentsInStreets().size() * 100 << "%)" << std::endl;
	}
	
	if(global::execOptions.showProgressBar) {
		std::cout << std::endl;
	}
	//
	// Se ajustan las reglas de los agentes
	//
	*global::serverLog  << "Ajustando reglas de los agentes... " <<  std::endl;
	_env->adjustAgentsRules();

	auto end = std::chrono::system_clock::now(); //Measure Time
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	global::simOutputs.timeExec.calibration += elapsed.count();
	

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

	if(global::execOptions.showProgressBar) {
		std::cout << std::endl;
	}
	
	//exit(EXIT_SUCCESS);
}

////////////////////////////////////////////////////////
//	Simulator::run()
//
void Simulator::run()
{
	std::string snitchServerURL = global::params.snitchServer.URL;
	json        similarSims;
	
	utils::radius_t seekRadius     = global::params.snitchServer.seekRadius; 
	uint32_t        xsteps         = global::params.snitchServer.xsteps;
	uint32_t        cutOff         = global::params.snitchServer.cutOff;
	utils::radius_t seekRadiusHMap = global::params.snitchServer.seekRadiusHMap;
	uint32_t        cutoffHMap     = global::params.snitchServer.cutoffHMap;
	
	global::params.sampling.level = _fsettings["samplingLevel"];
	
	global::simOutputs.logs.usePhone.resize(_duration+1);
	global::simOutputs.logs.velocity.resize(_env->getTotalAgents());
	
	////////////////////////////////////////////////////////////////////////
	// Características de las zonas iniciales y seguras
	//
	json sdata_zonesInfo;
	_env->getZonesInfo(sdata_zonesInfo);
	
	*global::serverLog  << "Simulando..." << std::endl;
	
	// Lanza el thread del watchDog
	std::string dirTodDelete = _heatMapPath;
	_simInExec = true;
	std::thread watchDogThread(&Simulator::watchDog, this,  global::params.watchDog.initialWaitTime, global::params.watchDog.deltaTime, global::params.watchDog.thresTime, dirTodDelete);
	
	
	ProgressBar pg;
	pg.start(_duration-1);
	//*global::serverLog  << "Simulando... tick=0" << std::endl;
	//////////////////////////////////////////////
	//Tiempo 0 equivale a las posiciones iniciales
	global::currTimeSim = 0;
	
	if( global::execOptions.agentsOut ) {
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
	
	if( global::params.modelsEnable.debris ){
		this->saveStateDebris();
	}
	
	//////////////////////////////////////////////////////////////////////
	// Ciclo de simulación
	utils::Timer<std::chrono::milliseconds> timerSimAprox;
	std::string lastHeatMapFilePath = "";
	std::vector<std::string> heatMapFiles;
	
	Environment::floodParams_t floodParams = _env->getFloodParams();
	
	timerSimAprox.start();
	//std::cout << "Simulando... tick>0" << std::endl;
	for(global::currTimeSim = 1; global::currTimeSim <= _duration; global::currTimeSim++) {

		if(global::currTimeSim % _interval == 0){
			std::stringstream p;
			double progress = (double)global::currTimeSim / (double)_duration * 100.0;

			p << "Tick = " << global::currTimeSim  << " / " << _duration << " ";
			p << std::setprecision(2) << std::fixed << "[" << progress << "%]";

			if( global::currTimeSim < _duration) {
				p << "...";
			}

			*global::serverLog << p.str() << std::endl;
		}

		//if(global::execOptions.showProgressBar) {
		//	pg.update(global::currTimeSim);
		//}

		utils::Timer<std::chrono::milliseconds> timerUpdates;
		timerUpdates.start();

		_env->updateQuads();
		// intervalo de tiempo que se demora en actualizar los cuadrantes
		auto deltaTimeQuads = timerUpdates.curr();
		
		_env->updateAgents();
		// intervalo de tiempo que se demora en actualizar los agentes
		auto deltaTimeAgents = timerUpdates.curr();

		// intervalo que se demora un ciclo de simulación, sin considerar I/O
		auto deltaTimeExecSim = deltaTimeQuads + deltaTimeAgents;

		// tiempo a que tomada la simulación
		global::simOutputs.timeExec.simulation += deltaTimeExecSim;
		
		std::stringstream  stepDelayLog;
		stepDelayLog << global::currTimeSim   << ":" << deltaTimeQuads  << ":" << deltaTimeAgents << ":" << deltaTimeExecSim  << ":";
		stepDelayLog << global::simOutputs.timeExec.simulation;
		global::simOutputs.logs.stepDelay.push_back( stepDelayLog.str() );
		
		
		if(global::execOptions.agentsOut && ((global::currTimeSim % _interval) == 0)) {
			this->savePositionAgents();
			//this->saveStatePatchAgents();
		}

		if(_statsOut && ((global::currTimeSim % _statsInterval) == 0)) {
			_env->updateLogsStats();
		}
		
		if(_heatMapOut && ((global::currTimeSim % _heatMapInterval) == 0)){
			lastHeatMapFilePath = this->saveStatePatchAgents();
			heatMapFiles.push_back(lastHeatMapFilePath);
		}
		
		if(floodParams.enable  &&((global::currTimeSim % floodParams.sampleStateInterval) == 0)){
			if(floodParams.imagesEnable){
				// crea imagen PGM del estado de la inundación
				this->saveImgFlood();
			}
			
			if(floodParams.stateEnable){
				// crea archivo del estado de la inundación
				this->saveStateFlood();
			}
		}
		
		
		if( (global::currTimeSim % global::params.sampling.interval) == 0 && global::params.sampling.compareWithOthersSims){
			this->samplingSim();
			if(g_TSevacAll[global::currTimeSim] >= global::params.sampling.level){
				
				//std::cout << global::currTimeSim << ":" << g_TSevacAll[global::currTimeSim] << std::endl;
				
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
					*global::serverLog   << "Path to heatMap " << heatMapQryFilePath << " no exist." << std::endl;
				}
				options += "&heatMapFile= " + heatMapQryFilePath;
	
				//Timer<std::chrono::milliseconds> timer1;
				//timer1.start();				
				try{
					utils::restClient_get(snitchServerURL + "/search?" + options, similarSims);
				} catch(std::exception& e){
					//std::cerr << e.what() <<std::endl;
					*global::serverLog   << "Could not connect to snitch server " << snitchServerURL << std::endl;
				}
				timerSimAprox.stop();
				similarSims["timeSimAprox"] = timerSimAprox.elapsed();
				similarSims["timeExecSim"] = global::simOutputs.timeExec.simulation;
				//timer1.stop();				
				
				//global::params.sampling.level += 0.1;
				global::params.sampling.compareWithOthersSims = false;
				
			}
		} // Fin del proceso de muestreo y comparación con simulaciones anteriores.

	}
	//Fin ciclo de simulación
	*global::serverLog << std::endl;
	*global::serverLog << "Waiting watchDog to end" << std::endl;
	_execForMTX.lock();
	_simInExec = false;
	_execForMTX.unlock();
	
	watchDogThread.join();
	
	if(global::execOptions.showProgressBar) {
		std::cout << std::endl;
	}

	this->executionSummary();

	if( _statsOut) {
		this->makeStats();
	}
	
	////////////////////////////////////////////////////////
	//  Crear el archivo JSON de la animación de la simulación
	std::string zoneFileName = std::filesystem::path(_fsettings["input"]["zones"].get<std::string>()).filename();
	std::string cdUP = "../";
	json animacionConfig = {
		{"pathFileSim", cdUP + _fsettings["output"]["agents-path"].get<std::string>()},
		{"frameMin", 0},
		{"frameMax", _duration},
		{"frameStep", _interval},
		{"deltaT", global::params.deltaT},
		{"mapOffset", 0.01}, //Offset del borde mapa (0.01 ==> 800metros )
		{"pathFileDebrisState", cdUP + _fsettings["debrisParams"]["stateDir"].get<std::string>() + "/" + _fsettings["debrisParams"]["infoFile"].get<std::string>()},
		{"pathFileFloodStateDir", cdUP +  _fsettings["floodParams"]["stateDir"].get<std::string>()},
		{"criticaFloodlLevel", _fsettings["floodParams"]["criticalLevel"]},
		{"input",{
			{"zones", "input/"+ zoneFileName}
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
	samplingDataSim["totalAgents"]  = global::simOutputs.totalAgentsInSim;
	samplingDataSim["zonesInfo"]    = sdata_zonesInfo;
	samplingDataSim["nonEvacByAge"] = sdata_DistSafeZoneByAgeGroup;
	samplingDataSim["evacByZone"]   = sdata_EvacTimeByZone;
	json evcTime(global::simOutputs.logs.evacTime);
	samplingDataSim["evacTime"]     = evcTime;
	samplingDataSim["heatMapPath"] = _heatMapPath;
	

	//std::cout << "---------------JSON dump---------------------" << std::endl;
	//std::cout << "---------------------------------------------" << std::endl;
	//std::cout << std::setw(4) << samplingDataSim                 << std::endl;
	//std::cout << "---------------------------------------------" << std::endl;
	
	
	if(global::params.sampling.saveSimInDB){
		RestClient::Response r = RestClient::post(snitchServerURL + "/cities", "application/json", samplingDataSim.dump());
	
		if(r.code == 200){
			json response = json::object();
			response = json::parse(r.body);

			*global::serverLog  << "Body:" << response.dump(4) << std::endl;
		}
		else{
			if(r.code == COULDNT_CONNECT){
				*global::serverLog  << "Could not connect to REST server." << std::endl;
			}
			else{
				*global::serverLog  << "Error: "<< r.code << std::endl;
			}
		}
	}
	
	*global::serverLog  << std::endl;
	*global::serverLog  << "UUID sim                 : " << _uuidSim << "\n";
	*global::serverLog  << "Simulation descriptiom   : " << _fsettings["description"].get<std::string>() << "\n";
	*global::serverLog  << "Elapsed real sim. time   : " << global::simOutputs.timeExec.simulation << "\n";
	*global::serverLog  << "Elapsed aprox. sim. time : " << similarSims["timeExecSim"] << std::endl;
	
	if( global::params.sampling.compareWithOthersSims){
		*global::serverLog  << "==== Similar simulations  ====" << std::endl;
		*global::serverLog  << "Sampling level           : " << global::params.sampling.level << "\n";
		*global::serverLog  << "==== by Time Series ====" << std::endl;
		for(const auto& element : similarSims["TS"]["dataFound"]["simFounds"]){
			*global::serverLog  << element["distance"]         << ":";
			*global::serverLog  << element["uuid"]             << ":";
			*global::serverLog  << element["description"]      << "\n";
		}
	
		*global::serverLog  << "==== by Heat Maps ====" << std::endl;
		for(const auto& element : similarSims["HM"]["dataFound"]["simFounds"]){
			*global::serverLog  << element["distance"]         << ":";
			*global::serverLog  << element["uuid"]             << ":";
			*global::serverLog  << element["description"]      << "\n";
		}
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
	ss << std::setw( 10 ) << std::setfill( '0' ) << global::currTimeSim;

	std::string nameFile = ss.str() + "." + _filesimSufix ;
	std::string pathFile = _agentsPath + "/" + nameFile ;


	std::ostringstream ssStateAgents;
	ssStateAgents << "idAgent latitude longitude agentModel usePhone isAlive speed velocity_X velocity_y direction_X direction_Y\n";
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
		ssStateAgents << " " << agent->getUsingPhone();
		ssStateAgents << " " << agent->isAlive();
		ssStateAgents << " " << curSpeed;
		ssStateAgents << " " << curVel << " ";	
		ssStateAgents << " " << direction << "\n";

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
	ss << std::setw( 10 ) << std::setfill( '0' ) << global::currTimeSim;

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

/**
 * @brief Crea archivos de estado de inundación
 *
 * Cuando es invocada en un tick de simulación determinado (tck), esta
 * función crea un archivo asociado a dicho tick con los datos de los
 * patchs que están inundados. Cada línea tiene el siguiente formato:
 *
 *    <ID_patch> <latitud> <logitud> <nivel actual> <nivel máximo>
 *
 *  <latitud> <logitud>: coordenadas del centro del patch.
 *  <nivel actual>: nivel de inundación en el tick actual, en metros
 *  <nivel máximo>: nivel máximo de inundación según la carta de inundación del lugar.
 *          1: 1 metros
 *          2: 2 metros
 *          4: 4 metros
 *          6: 6 metros
 *         -1: >6 metros        
 *
 * @return (void)
 */
void Simulator::saveStateFlood()
{
	Environment::floodParams_t floodParams = _env->getFloodParams();
	
	std::ostringstream ss;
	ss << std::setw( 10 ) << std::setfill( '0' ) << global::currTimeSim;

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
	ss << std::setw( 10 ) << std::setfill( '0' ) << global::currTimeSim;

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


/**
 * @brief Al finalizar la simulación, guarda los costos temporales y espaciales de la ejecución
 *
 * Al finalizar la simulación, esta función un archivo con los costos relevantes de la ejecución:
 * a) crear los agentes, b) calibración de ellos en el mapa, c) simulación, 
 * d) memoria máxima utilizada por todo el simulador y e) memoria utilizada por los agentes.
 * Archivos creados:
 *      executionSummary[exp].txt
 * @return (void)
 */
void Simulator::executionSummary()
{
	std::string pathFile01;
	
	uint32_t maxMemory = this->getMaxMemory();
	
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
	
	ofs01 << _duration*global::params.deltaT << ":"
	      << _calibrationTime*global::params.deltaT << ":"
		  << global::params.deltaT << ":"
	      << _fsettings["agents"][0]["number"] << ":"
		  << _fsettings["agents"][1]["number"] << ":"
	      << global::simOutputs.timeExec.makeAgents  << ":"
	      << global::simOutputs.timeExec.calibration << ":"
	      << global::simOutputs.timeExec.simulation << ":"
	      << maxMemory << ":"
	      << global::simOutputs.agentsMem
	      << std::endl;
	
	ofs01.close();
}


/**
 * @brief Al finalizar la simulación, crea los archivos de salida
 *
 * Al finalizar la simulación, esta función crea los archivos con los datos de salida, que
 * serán consumidos por programas de análisis.
 * Archivos creados:
 *      zonesDensity[exp].txt
 *      usePhone[exp].txt
 *      deceasedAgents[exp].txt
 *      SIRpanic[exp].txt
 *      velocity[exp].txt
 *      summary[exp].txt
 *      stepDelay[exp].txt
 * @return (void)
 */
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
	for(auto& item : global::simOutputs.logs.zonesDensity) {
		
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
		uint32_t timeSim = tick * global::params.deltaT;
		
		if( _numExperiment >= 0){
			ofs01 << _numExperiment << ":";
		}
		
		ofs01 << std::to_string(timeSim) << ":" << std::to_string(global::simOutputs.logs.usePhone[tick]) << std::endl;
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
	for(auto& item : global::simOutputs.logs.deceasedAgents) {
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
	
	
	for(auto& item : global::simOutputs.logs.SIRpanic) {

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
		uint32_t timeSim = tick * global::params.deltaT;
	
		ofs01 << std::to_string(timeSim) << ":";
	}
	
	ofs01 << std::endl;
	
	ofs01 << "Coming soon..." <<std::endl;
	
	/*
	for(auto& fooAgent : _env->getAgents()){
		ofs01 << std::to_string(fooAgent->id()) << ":";
		ofs01 << std::to_string(fooAgent->model()) << ":" ;
		ofs01 << std::to_string(fooAgent->groupAge()) << ":";
		ofs01 << global::simOutputs.velocity[fooAgent->id()] << std::endl;
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
	std::uniform_real_distribution<> unifDistro(-(float)_interval*global::params.deltaT/2.0, (float)_interval*global::params.deltaT/2.0); //2020-10-09

	ofs01 << "id:model:groupAge:safeZone:distanceToTargetPos:responseTime:evacTime:initialZone:travelDistance:isAlive:timeLived" << std::endl;		
	for(auto& fooAgent : _env->getAgents()){
		if( _numExperiment >= 0){
			ofs01 << _numExperiment << ":";
		}
		
		//aleatorizar el tiempo de evacuación para que esté 
		//en un rango tevac +/- (_interval)/2
		double evacTime = fooAgent->getEvacuationDataStruct().evacuationTime;
		if( evacTime >  (float)_interval*global::params.deltaT/2.0 ){ //2020-10-09
			evacTime +=  unifDistro(_randomDevice) ; 
		}
		
		//guardar los tiempos de evacuacion para 
		//enviarlos al back-end.
		global::simOutputs.logs.evacTime.push_back(evacTime);
		
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

	std::stringstream buff;
	
	if( _numExperiment >= 0){
		buff << "numExperiment:" ;
	}
	
	buff << "time:DeltaTimeQuads:DeltaTimeAgents:DeltaTimeExecSim:timeExecSim\n";		

	for(auto& item : global::simOutputs.logs.stepDelay) {

		if( _numExperiment >= 0){
			buff << _numExperiment << ":";
		}
		
		buff << item << "\n";
	}
	ofs01 << buff.str();
	ofs01.close();
}

/**
 * @brief Realiza el muestreo de la simulación en curso
 * @return (void)
 */
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
	// de simulación actual 'global::currTimeSim'
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
		pNonEvac = (float)nonEvacNumber / (float)global::simOutputs.totalAgentsInSim;

		g_sdataDistSafeZoneByAgeGroup[std::to_string(global::currTimeSim)][ageGroup] = std::make_tuple(nonEvacNumber, pNonEvac, minDist, maxDist, meanDist);
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
	// el tiempo de simulación actual 'global::currTimeSim'
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
		pEvac = (float)evacNumber / (float)global::simOutputs.totalAgentsInSim;
		
		g_sdataEvacTimeByZone[std::to_string(global::currTimeSim)][zoneID] = std::make_tuple(evacNumber, pEvac, minEvacTime, maxEvacTime, meanEvacTime);
	}
	
	
	// Crear las TS para que la simulación en curso se puede comparar con 
	// las simulaciones almacenadas.
	g_TSevacAll[global::currTimeSim] = evacAgents / (double)global::simOutputs.totalAgentsInSim;

}


/**
 * @brief Determina el sufijo de una archivo
 *
 *	Si la ejecución no pertenece a un conjunto de experimentos, retorna
 *  sólo la extensión ".txt".
 *  En caso contrario, retorna "%.5d.txt"
 *
 * @return _numExperiment == -1 ? ".txt" : "%.5d.txt" _numExperiment
 */
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


/**
 * @brief Memoria máxima utilizada
 *
 * Retorna la cantidad máxima de memoria utilizada por el 
 * simulador
 * @return Memoria máxima utilizada (KB)
 */
uint32_t Simulator::getMaxMemory()
{
	struct rusage r_usage;
	getrusage(RUSAGE_SELF,&r_usage);
	//https://www.gnu.org/software/libc/manual/html_node/Resource-Usage.html
	uint32_t maxMemory = r_usage.ru_maxrss;//KB

	return(maxMemory);
}


/**
 * @brief Monitoreo de avance de la simulación
 * Vigila el avance de la simulación cada 'deltaTime', desde el intante 'initialWaitTime'. Si no avanza por 'thresTime' segundos, elimina la simulación actual. 
 * Estos tiempos se definen en la estructura 'WatchDog_s' en en archivo 'global.cc'
 *      global::params.watchDog.initialWaitTime
 *      global::params.watchDog.deltaTime
 *      global::params.watchDog.thresTime
 * @param initialWaitTime Instante de tiempo donde comienza el monitoreo
 * @param deltaTime Intervalo de tiempo que el thread despierta.
 * @param thresTime Cantidad de segundos que se considera que la simulación ya no avanza y se debe abortar.
 * @param dirTodDelete Directorio que se debe eliminar cuando la simulación aborte.
 * @return (void)
 */
void Simulator::watchDog(uint32_t initialWaitTime,  uint32_t deltaTime,  uint32_t thresTime, std::string dirTodDelete)
{
	uint32_t tickOld = 0;
	uint32_t tickNew = 0;	
	uint32_t timesBlocked = 0;

	std::ofstream pidFile;

	pidFile.open(_pidFilePath);
	pidFile << getpid();
	pidFile.close();

	std::this_thread::sleep_for(std::chrono::seconds(initialWaitTime));
    while (1){
		tickNew = global::currTimeSim;
		
		_execForMTX.lock();
		if(!_simInExec){
			*global::serverLog  << "ciclo terminado" << std::endl;
			try {
				std::filesystem::remove(_pidFilePath);
				*global::serverLog  << "Deleted " << _pidFilePath << " file.\n";
			} catch (const std::filesystem::filesystem_error& e) {
				*global::serverLog  << "Error: " << e.what() << "\n";
			}
			break;
		}
		_execForMTX.unlock();
		
		if(tickNew == tickOld){
			uint32_t blockedTime;
			
			timesBlocked++;
			blockedTime = timesBlocked*deltaTime;
			if(timesBlocked*deltaTime >= thresTime){
				*global::serverLog  << "ATENCIÓN: El ciclo no ha avanzado por " << blockedTime << " s. Forzando salida." << std::endl;
				
				if (std::filesystem::exists(dirTodDelete) && std::filesystem::is_directory(dirTodDelete)) {
					try {
						std::filesystem::remove(dirTodDelete);
						*global::serverLog  << "Deleted " << dirTodDelete << " directory.\n";
					} catch (const std::filesystem::filesystem_error& e) {
						*global::serverLog  << "Error: " << e.what() << "\n";
					}
					try {
						std::filesystem::remove(_pidFilePath);
						*global::serverLog  << "Deleted " << _pidFilePath << " file.\n";
					} catch (const std::filesystem::filesystem_error& e) {
						*global::serverLog  << "Error: " << e.what() << "\n";
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















