#include <json.hpp>
#include <glob.hh>
#include <simulator.hh>
#include <checkArgs.hpp>
#include <utils.hh>


bool  g_showProgressBar;
bool  g_panicModelEnable;
bool  g_elevationModelEnable;
bool  g_elevationPatchDataValid;
bool  g_debrisModelEnable;
bool  g_floodModelEnable;
bool  g_agentsOut;
float g_closeEnough;
float g_randomWalkwayRadius;
float g_attractionRadius;

uint32_t g_currTimeSim;
uint32_t g_AgentsMem;
float    g_deltaT;
uint32_t g_totalAgentsInSim;

uint32_t g_timeExecMakeAgents;
uint32_t g_timeExecCal;
uint32_t g_timeExecSim;

std::string g_baseDir;

std::vector<std::string> g_logZonesDensity;
std::vector<uint32_t>    g_logUsePhone;
std::vector<std::string> g_logVelocity;
std::vector<std::string> g_logSIRpanic;
std::vector<std::string> g_logDeceasedAgents;
std::ostringstream       g_logStepDelay;

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

std::vector<double> g_evacTime;

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

int main(int argc,char** argv)
{

	json settings;
	json zones;

	// Adquirir parámetros de entrada
	std::shared_ptr<CheckArgs> argumentos = std::make_shared<CheckArgs>(argc, argv);

	// Carga el archivo de configuración JSON en settings.
	std::string fileConfig = argumentos->getArgs().fileConfig;
	std::ifstream ifs;
	ifs.open(fileConfig, std::ifstream::in);
	if( ifs.fail() ) {
		std::cerr << "Error in open file: "<< argumentos->getArgs().fileConfig << std::endl;
		ifs.close();
		exit(EXIT_FAILURE);
	}
	ifs >> settings;
	ifs.close();

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
	std::string elevationFile = argumentos->getArgs().elevationFile;
	uint32_t agentsResidentsNumber = argumentos->getArgs().agentsResidentsNumber;
	uint32_t agentsVisitorsNumber  = argumentos->getArgs().agentsVisitorsNumber;
	std::string description     = argumentos->getArgs().description;
	uint32_t numThreads         = argumentos->getArgs().numThreads ;
	std::string outputDirectory = argumentos->getArgs().outputDirectory;
	int32_t numExperiment       = argumentos->getArgs().numExperiment ;
	double samplingLevel        = argumentos->getArgs().samplingLevel ;
	bool patchCoords            = argumentos->getArgs().patchCoords ;
	
	
	// Por omisión, es falsa, a menos que se invoque con el parametro -E
	settings["patchCoords"] = patchCoords;
	
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
	
	try{
		g_baseDir = settings["baseDirSim"].get<std::string>();
	}
	catch (json::exception &e) {
		std::cerr << "Item 'baseDirSim' not defined in file " + fileConfig << std::endl;
		std::cerr << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
	

	settings["elevationPatchDataValid"] = true;
	std::string inputBaseDir = settings["input"]["directory"].get<std::string>();
	try {
		map_osrm             = g_baseDir + inputBaseDir + settings["input"]["map"].get<std::string>();
		zones_file           = g_baseDir + inputBaseDir + settings["input"]["zones"].get<std::string>();
		elevationFile        = g_baseDir + inputBaseDir + elevationFile;
	} catch (json::exception &e) {
		std::cerr << "Error in get action from 'input' section in <config.json>:" << std::endl;
		std::cerr << e.what() << std::endl;
		exit(EXIT_FAILURE);
	};
	settings["input"]["elevationPatchData"] = elevationFile;
	
	if(settings["elevationModelEnable"]){
		if( !std::filesystem::exists(elevationFile)){
			settings["elevationPatchDataValid"] = false;
			settings["elevationModelEnable"] = false;
		
			std::cout << "\x1B[1;37m";
			std::cout << "Los datos geográficos de elevación no están disponibles.\nEl archivo:" << std::endl;
			std::cout << elevationFile << " no existe.";
			std::cout << "El modelo de elevación no será activado." << std::endl;
			std::cout << "\x1B[0m" << std::endl;
		}
	}
	
	

	// El nombre archivo osrm es utilizado por la
	// clase Router. Se revisa que efectivamente exista y
	// se pueda leer.
	ifs.open(map_osrm,std::ifstream::in);
	if(ifs.fail()) {
		std::cerr << "Error in open file: "<<  map_osrm << std::endl;
		ifs.close();
		exit(EXIT_FAILURE);
	}
	ifs.close();

	// Carga los datos geográficos del archivo geojson 'zones_file'
	// en la variable 'zones'
	loadDataFrom(zones_file, zones);
	
	std::cout << "\x1B[0;90m";
	std::cout << "Revisando estructura del archivo de zonas:\n\t '" <<  zones_file  << "' ... " ;
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
		std::cout << "OK" << std::endl;
	}
	else{
		std::cout << "Error.\nEl archivo debe tener definida al menos una zona inicial y una zona segura:\n";
		std::cout << "Zonas iniciales: " << initialZones << std::endl;
		std::cout << "Zonas seguras  : " << safeZones    << std::endl;
		std::cout << "\x1B[0m" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::cout << "\x1B[0m" << std::endl;
	
	
	std::cout << "\x1B[0;37m";
	std::cout << "Simmulation parameters:" << std::endl;
	std::cout << "\tDescription  : " << std::endl;
	std::cout << "\t\t" << settings["description"].get<std::string>() << std::endl;
	std::cout << "\tAgents:" << std::endl;
	std::cout << "\t\tresidents  : " << settings["agents"][0]["number"].get<uint32_t>() << std::endl;
	std::cout << "\t\tvisitors   : " << settings["agents"][1]["number"].get<uint32_t>() << std::endl;
	std::cout << "\tSamplingLevel: " << settings["samplingLevel"] << std::endl;
	std::cout << "Flood model:" << std::endl;
	std::cout << "\tEnable: " << std::boolalpha << settings["floodModelEnable"] << std::endl;
	std::cout << "Density model:" << std::endl;
	std::cout << "\tEnable: " << settings["densityModelEnable"] << std::endl;
	std::cout << "Panic model:" << std::endl;
	std::cout << "\tEnable :" << settings["panicModelEnable"] << std::endl;
	std::cout << "Elevation model:" << std::endl;
	std::cout << "\tEnable :" << settings["elevationModelEnable"] << std::endl;
	std::cout << "Debris model:" << std::endl;
	std::cout << "\tEnable :" << settings["debrisModelEnable"] << std::endl;
	std::cout << "\x1B[0m" << std::endl;
	

	//Reset counters
	g_timeExecMakeAgents = 0;
	g_timeExecCal        = 0;
	g_timeExecSim        = 0;

	
	Simulator sim(settings, zones, map_osrm);

	omp_set_num_threads( settings["threads"].get<uint32_t>() );

	sim.calibrate();
	sim.run();


	return(EXIT_SUCCESS);
}
