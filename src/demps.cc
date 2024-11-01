#include <demps.hh>


/*std::map<std::string, model_t> model_map = {
	{"ShortestPath",   ShortestPath},
	{"FollowTheCrowd", FollowTheCrowd},
	{"RandomWalkway",  RandomWalkway}
};*/

std::map<std::string, model_t> model_map = {
	{"Residents",   Residents},
	{"Visitors", Visitors_II}
};


////////////////////////////////////////////////////////////////////////////////////////////////
// versión con los índices en std::string para permitir la conversión a JSON en forma automática
//
//        Ts                      Ak                      N(k,s)    %N     min(k) max(k) meanDist
std::map< std::string,  std::map< std::string, std::tuple<uint32_t, float, float, float, float> > > g_sdataDistSafeZoneByAgeGroup;
//        Ts                      Zn                      E(n,s)   %E(n,s) tevac_min  tevac_max  tevac_mean
std::map< std::string,  std::map< std::string, std::tuple<uint32_t, float, float    , float    , float      > > > g_sdataEvacTimeByZone;


////////////////////////////////////////////////////////////////////////////////////////////////
//  Series de tiempo para comparar la simulación en curso con simulaciones
//  anteriores
utils::timeSerie_t g_TSevacAll;

void loadDataFrom(std::string fileIn, json& dataOut)
{

	std::ifstream ifs;

	ifs.open(fileIn,std::ifstream::in);
	if(ifs.fail()) {
		std::cerr << "Open error in file:"<<  fileIn << std::endl;
		exit(EXIT_FAILURE);
	}

	ifs >> dataOut;
	ifs.close();

	if(dataOut.empty()) {
		std::cerr << "Can't load data from:"<<  fileIn << std::endl;
		exit(EXIT_FAILURE);
	}
}

void makeDefaultConfigFile(){
	std::stringstream buff;

	buff << "[params]\n";
	buff << "offsetMap=500\n";
	buff << "animationDir=" << global::params.configDempsDir <<  "animation/\n";
	buff << "animationFile=animation.html\n";
	buff << "\n";
	buff << "[params.sampling]\n";
	buff << "compareWithOthersSims=false\n";
	buff << "level=0.5\n";
	buff << "saveSimInDB=false\n";
	buff << "\n";
	buff << "[params.watchdog]\n";
	buff << "initialWaitTime=60\n";
	buff << "deltaTime=60\n";
	buff << "thresTime=30\n";
	buff << "pidFile=demps.pid\n";
	buff << "\n";
	buff << "[params.snitchServer]\n";
	buff << "URL=http://snitch:10667/v1/snitch/api\n";
	buff << "seekRadius=0.5\n";
	buff << "xsteps=10\n";
	buff << "cutOff=10\n";
	buff << "seekRadiusHMap=5\n";
	buff << "cutoffHMap=10\n";
	buff << "\n";
	buff << "[params.elevationServer]\n";
	buff << "URL=http://oes:10666/api/v1\n";
	buff << "coorTest=-33.144995,-71.568655\n";

	std::ofstream ofs;

	std::string configConfigDirPath = global::params.configDempsDir + global::params.configDir;
	if( !std::filesystem::exists( configConfigDirPath  )){
		std::filesystem::create_directories( configConfigDirPath );
	}

	std::string configFilePath = configConfigDirPath + global::params.configFile;

	ofs.open(configFilePath);
	if(ofs.fail()) {
		std::cerr << "Failed to create file '" << configFilePath << "'\n";
		exit(EXIT_FAILURE);
	}

	ofs << buff.str() << std::endl;
	ofs.close();


}

int main(int argc,char** argv)
{

	json settings;
	json zones;


	// Inicia el log
	global::serverLog = new StreamLog("demps", LOG_LOCAL1);
	global::serverLog->toCOUT(global::execOptions.logToCOUT);

	// Adquirir parámetros de entrada
	std::shared_ptr<CheckArgs> argumentos = std::make_shared<CheckArgs>(argc, argv);

	bool makeConfig = argumentos->getArgs().makeConfig;

	std::string configFilePath = global::params.configDempsDir + global::params.configDir + global::params.configFile;

	if(makeConfig){
		*global::serverLog  << "Make default configuration in "<< configFilePath << std::endl;
		makeDefaultConfigFile();
		exit(EXIT_SUCCESS);
	}
	
	// Verificar que el archivo de  configuración 'configFilePath' existe
	if( !std::filesystem::exists(configFilePath) ){
		*global::serverLog << "Error.\nConfig file " << configFilePath << " no exists.\n";
		//*global::serverLog << "Execute '" <<  argv[0] << " --makeconfig' for create a default configuration\n";
		//*global::serverLog << std::endl;
		//exit(EXIT_FAILURE);
		*global::serverLog  << "Make default configuration in "<< configFilePath << std::endl;
		makeDefaultConfigFile();
	}

	// Cargar variables globales desde el archivo de configuración 'configFilePath'
	iniLib::INIReader dempsConfig{configFilePath};
	try{
		global::params.offsetMap                      = dempsConfig.Get<uint32_t>("params","offsetMap");
		global::params.animationDir                   = dempsConfig.Get<std::string>("params","animationDir");
		global::params.animationFile                  = dempsConfig.Get<std::string>("params","animationFile");
		global::params.sampling.compareWithOthersSims = dempsConfig.Get<bool>("params.sampling","compareWithOthersSims");
		global::params.sampling.level                 = dempsConfig.Get<double>("params.sampling","level");
		global::params.sampling.saveSimInDB           = dempsConfig.Get<bool>("params.sampling","saveSimInDB");
		global::params.watchDog.initialWaitTime       = dempsConfig.Get<uint32_t>("params.watchdog","initialWaitTime");
		global::params.watchDog.deltaTime             = dempsConfig.Get<uint32_t>("params.watchdog","deltaTime");
		global::params.watchDog.thresTime             = dempsConfig.Get<uint32_t>("params.watchdog","thresTime");
		global::params.watchDog.pidFile               = dempsConfig.Get<std::string>("params.watchdog","pidFile");
		global::params.snitchServer.URL               = dempsConfig.Get<std::string>("params.snitchServer","URL");
		global::params.snitchServer.seekRadius        = dempsConfig.Get<double>("params.snitchServer","seekRadius");
		global::params.snitchServer.xsteps            = dempsConfig.Get<uint32_t>("params.snitchServer","xsteps");
		global::params.snitchServer.cutOff            = dempsConfig.Get<uint32_t>("params.snitchServer","cutOff");
		global::params.snitchServer.seekRadiusHMap    = dempsConfig.Get<double>("params.snitchServer","seekRadiusHMap");
		global::params.snitchServer.cutoffHMap        = dempsConfig.Get<uint32_t>("params.snitchServer","cutoffHMap");
		global::params.elevationServer.URL            = dempsConfig.Get<std::string>("params.elevationServer","URL");
		global::params.elevationServer.coorTest       = dempsConfig.Get<std::string>("params.elevationServer","coorTest");
	}
	catch(std::exception& e){
		*global::serverLog << "Error in get action from file '" << configFilePath << "'\n";
		*global::serverLog << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	// Carga el archivo de configuración JSON en settings.
	std::string fileConfig = argumentos->getArgs().fileConfig;
	std::ifstream ifs;
	
	*global::serverLog  << "Load configuration from "<< fileConfig << std::endl;
	ifs.open(fileConfig, std::ifstream::in);
	if( ifs.fail() ) {
		*global::serverLog  << "Error in open file: "<< fileConfig << std::endl;
		ifs.close();
		exit(EXIT_FAILURE);
	}
	ifs >> settings;
	ifs.close();

	*global::serverLog << "\n\n";
	*global::serverLog << "\t****************************\n";
	*global::serverLog << "\t*  Starting new simulation *\n";
	*global::serverLog << "\t****************************\n";
	*global::serverLog << std::endl;

	//En el caso que existan paramentros de entrada,
	//sobreescribe los valores del json settings
	uint32_t duration    = argumentos->getArgs().duration;
	int32_t floodModel   = argumentos->getArgs().floodModel;
	int32_t panicModel   = argumentos->getArgs().panicModel;
	int32_t emotionThreshold = argumentos->getArgs().emotionThreshold;
	int32_t densityModel = argumentos->getArgs().densityModel;
	int32_t debrisModel  = argumentos->getArgs().debrisModel;
	int32_t debrisRatio  = argumentos->getArgs().debrisRatio;
	int32_t elevationModel  = argumentos->getArgs().elevationModel;
	//std::string elevationFile = argumentos->getArgs().elevationFile;
	uint32_t agentsResidentsNumber = argumentos->getArgs().agentsResidentsNumber;
	uint32_t agentsVisitorsNumber  = argumentos->getArgs().agentsVisitorsNumber;
	std::string description     = argumentos->getArgs().description;
	uint32_t numThreads         = argumentos->getArgs().numThreads ;
	std::string outputDirectory = argumentos->getArgs().outputDirectory;
	int32_t numExperiment       = argumentos->getArgs().numExperiment ;
	double samplingLevel        = argumentos->getArgs().samplingLevel ;
	//bool patchCoords            = argumentos->getArgs().patchCoords ;
	

	// Falta colocar este campo en el archivo de configuración y en el formulario web
	settings["output"]["anim-path"] = "animation/"; 

	
	// por omisión floodModel=-1. Manda el archivo de configuración
	if(floodModel == 0){
		settings["floodModelEnable"] = false;
	}
	else if(floodModel == 1){
		settings["floodModelEnable"] = true;
	}
	
	// por omisión panicModel=-1. Manda el archivo de configuración
	if(panicModel == 0){
		settings["panicModelEnable"] = false;
		settings["panicModelManageByParameter"] = true;
	}
	else if(panicModel > 0){
		// el valor de 'floodModel' es el nivel 'emotionThreshold'
		settings["panicModelEnable"] = true;
		settings["emotionThreshold"] = emotionThreshold;
		settings["panicModelManageByParameter"] = true;
	}
	else{
		settings["panicModelManageByParameter"] = false;
	}
	
	// por omisión densityModel=-1. Manda el archivo de configuración
	if(densityModel == 0){
		settings["densityModelEnable"] = false;
	}
	else if(densityModel == 1){
		settings["densityModelEnable"] = true;
	}
	
	// por omisión debrisModelEnable=-1. Manda el archivo de configuración
	settings["debrisParams"]["debrisRatio"] = (double)debrisRatio;
	settings["debrisParams"]["stateDir"] = "debrisState";
	settings["debrisParams"]["infoFile"] = "infoFile.txt";
	if(debrisModel == 0){
		settings["debrisModelEnable"] = false;
	}
	else if(debrisModel == 1){
		settings["debrisModelEnable"] = true;
	}
	
	// por omisión elevationModel=-1. Manda el archivo de configuración
	if(elevationModel == 0){
		settings["elevationModelEnable"] = false;
	}
	else if(elevationModel == 1){
		settings["elevationModelEnable"] = true;
	}
	
	
	if(duration > 0) {
		settings["duration"] = duration;
	}
	if(agentsResidentsNumber > 0) {
		settings["agents"][0]["number"] = agentsResidentsNumber;
	}
	if(agentsVisitorsNumber > 0) {
		settings["agents"][1]["number"] = agentsVisitorsNumber;
	}
	if(description != "null") {
		settings["description"] = description;
	}
	if(numThreads > 0) {
		settings["threads"] = numThreads;
	}
	if(outputDirectory != "") {
		settings["output"]["directory"] = outputDirectory;
	}
	if(samplingLevel > 0) {
		settings["samplingLevel"] = samplingLevel;
	}
	else{
		settings["samplingLevel"] = 0.5;
	}
	
	
	//Si es >=0, entonces se asume que se están haciendo simulaciones
	//
	settings["numExperiment"] = numExperiment;

	// En base a los datos de la sección "input" del archivo
	// de configuración, carga las rutas de los archivos
	// que utiliza el simulador.
	// Debido a que están declarados con rutas relativas al item 'baseDirSim',
	// se le agrega la ruta correspondiente.

	std::string map_osrm;
	std::string zones_file;

	try {
		if(settings["output"]["directory"].get<std::string>()[0] != '/'){
			// La ruta del directorio de salida debe ser absoluta
			*global::serverLog  << "Output Directory must be absolute path." << std::endl;
			exit(EXIT_FAILURE);
		}

		if( settings["input"]["map"].get<std::string>()[0] != '/' ){
			// La ruta del archivo map.osrm debe ser absoluta
			*global::serverLog  << "Path of map.osrm must be absolute path. " << std::endl;
			exit(EXIT_FAILURE);
		}

		if( !std::filesystem::exists(settings["input"]["map"].get<std::string>())){
			*global::serverLog  << "Path to map.osrm file " << settings["input"]["map"].get<std::string>() << " no exist." << std::endl;
			exit(EXIT_FAILURE);
		}

		if( settings["input"]["zones"].get<std::string>()[0]  != '/' ){
			// La ruta del archivo de zonas debe ser absoluta
			*global::serverLog  << "Path of zones file must be absolute path. " << std::endl;
			exit(EXIT_FAILURE);
		}

		if( !std::filesystem::exists(settings["input"]["zones"].get<std::string>())){
			*global::serverLog  << "Path to zones file " << settings["input"]["zones"].get<std::string>() << " no exist." << std::endl;
			exit(EXIT_FAILURE);
		}


		// El campo "input.map" contiene una ruta absoluta
		map_osrm = settings["input"]["map"].get<std::string>();

		// El campo "input.zones" contiene una ruta absoluta
		zones_file = settings["input"]["zones"].get<std::string>();

	} catch (json::exception &e) {
		*global::serverLog << "Error in get action from 'input' section in <config.json>:" << std::endl;
		*global::serverLog << e.what() << std::endl;
		exit(EXIT_FAILURE);
	};
	
	if(settings["elevationModelEnable"]){
		json geoInfoTest;
		std::string req = global::params.elevationServer.URL + "/lookup?locations=" + global::params.elevationServer.coorTest;

		*global::serverLog << "Verificando servidor de elevación:" <<  global::params.elevationServer.URL << "\n";
		*global::serverLog << "Solicitud: " << req <<  std::endl;

		try{
			utils::restClient_get(req, geoInfoTest);

			assert(geoInfoTest["results"][0]["elevation"].get<int>() >= 0);
			*global::serverLog << "Solicitud: " << req << "\n";
			*global::serverLog << "El servidor " << global::params.elevationServer.URL << " entrega datos de elevación:\n";
			*global::serverLog << geoInfoTest.dump(4) << std::endl;

		} catch(std::exception& e){
			settings["elevationModelEnable"] = false;

			*global::serverLog << "Los datos geográficos de elevación no están disponibles.\n";
			*global::serverLog << "\tNo es posible acceder al servidor de elevación " << global::params.elevationServer.URL  << "\n";
			*global::serverLog << "\tEl modelo de elevación no será activado." << std::endl;
		}
	}
	
	

	// El nombre archivo osrm es utilizado por la
	// clase Router. Se revisa que efectivamente exista y/
	// se pueda leer.
	ifs.open(map_osrm,std::ifstream::in);
	if(ifs.fail()) {
		*global::serverLog << "Error in open file: "<<  map_osrm << std::endl;
		ifs.close();
		exit(EXIT_FAILURE);
	}
	ifs.close();

	// Carga los datos geográficos del archivo geojson 'zones_file'
	// en la variable 'zones'
	loadDataFrom(zones_file, zones);
	
	//*global::serverLog << "\x1B[0;90m";
	*global::serverLog << "Revisando estructura del archivo de zonas:\n\t '" <<  zones_file  << "' ... " ;
	uint32_t initialZones = 0;
	uint32_t safeZones    = 0;
	for(const auto& feature : zones["features"]) {
		std::string zoneType = feature["properties"]["zoneType"].get<std::string>();
		if(zoneType == "initial"){
			initialZones++;
		}
		else if(zoneType == "safe"){
			safeZones++;
		}
	}
	
	if(initialZones > 0 && safeZones > 0){
		*global::serverLog << "OK" << std::endl;
	}
	else{
		*global::serverLog << std::endl;
		*global::serverLog << "Error.\nEl archivo debe tener definida al menos una zona inicial y una zona segura:\n";
		*global::serverLog << "Zonas iniciales: " << initialZones << "\n";
		*global::serverLog << "Zonas seguras  : " << safeZones    << "\n";
		*global::serverLog << std::endl;
		exit(EXIT_FAILURE);
	}
	//*global::serverLog << "\x1B[0m" << std::endl;
	

	*global::serverLog << "Simmulation parameters:\n" ;
	*global::serverLog << "\tDescription  : \n" ;
	*global::serverLog << "\t\t" << settings["description"].get<std::string>() << "\n";
	*global::serverLog << "\tAgents:\n" ;
	*global::serverLog << "\t\tOutput enable: " << std::boolalpha << settings["output"]["agents-out"].get<bool>() << "\n";
	*global::serverLog << "\t\tresidents    : " << settings["agents"][0]["number"].get<uint32_t>() << "\n";
	*global::serverLog << "\t\tvisitors     : " << settings["agents"][1]["number"].get<uint32_t>() << "\n";
	*global::serverLog << "\tSamplingLevel: " << settings["samplingLevel"] << "\n";
	*global::serverLog << "Flood model:\n";
	*global::serverLog << "\tEnable: " << std::boolalpha << settings["floodModelEnable"] << "\n";
	*global::serverLog << "Density model:\n" ;
	*global::serverLog << "\tEnable: " << settings["densityModelEnable"] << "\n";
	*global::serverLog << "Panic model:\n" ;
	*global::serverLog << "\tEnable :" << settings["panicModelEnable"] << "\n";
	*global::serverLog << "Elevation model:\n" ;
	*global::serverLog << "\tEnable :" << settings["elevationModelEnable"] << "\n";
	*global::serverLog << "Debris model:\n";
	*global::serverLog << "\tEnable :" << settings["debrisModelEnable"] << "\n";
	*global::serverLog << std::endl;
	

	//Reset counters
	global::simOutputs.timeExec.makeAgents  = 0;
	global::simOutputs.timeExec.calibration = 0;
	global::simOutputs.timeExec.simulation  = 0;

	
	Simulator sim(settings, zones, map_osrm);

	omp_set_num_threads( settings["threads"].get<uint32_t>() );

	sim.calibrate();
	sim.run();


	return(EXIT_SUCCESS);
}
