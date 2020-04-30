#include <json.hpp>
#include <glob.hh>
#include <simulator.hh>
#include <checkArgs.hpp>

bool  g_showProgressBar;
float g_closeEnough;
float g_randomWalkwayRadius;
float g_attractionRadius;

uint32_t g_epochInitSim;
uint32_t g_currTimeSim;

float    g_deltaT;

uint32_t g_AgentsMem;

uint32_t g_timeExecMakeAgents;
uint32_t g_timeExecCal;
uint32_t g_timeExecSim;

std::string g_baseDir;

std::vector<std::string> g_logZonesDensity;
std::vector<uint32_t> g_logUsePhone;

/*std::map<std::string, model_t> model_map = {
	{"ShortestPath",   ShortestPath},
	{"FollowTheCrowd", FollowTheCrowd},
	{"RandomWalkway",  RandomWalkway}
};*/

std::map<std::string, model_t> model_map = {
	{"Residents",   Residents},
	{"Visitors", Visitors_II}
};

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
	json area_zone;
	json initial_zones;
	json reference_zones;

	// Adquirir parámetros de entrada
	std::shared_ptr<checkArgs> argumentos = std::make_shared<checkArgs>(argc, argv);

	// Carga el archivo de configuración JSON en settings.
	std::ifstream ifs;
	ifs.open(argumentos->getArgs().fileConfig, std::ifstream::in);
	if( ifs.fail() ) {
		std::cerr << "Error in open file: "<< argumentos->getArgs().fileConfig << std::endl;
		ifs.close();
		exit(EXIT_FAILURE);
	}
	ifs >> settings;
	ifs.close();

	//En el caso que existan paramentros de entrada,
	//sobreescribe los valores del json settings
	uint32_t duration     = argumentos->getArgs().duration;
	uint32_t agentsNumber = argumentos->getArgs().agentsNumber;
	uint32_t numThreads   = argumentos->getArgs().numThreads ;
	std::string outputDirectory = argumentos->getArgs().outputDirectory;
	int32_t numExperiment = argumentos->getArgs().numExperiment ;

	if(duration > 0) {
		settings["duration"] = duration;
	}
	if(agentsNumber > 0) {
		settings["agents"][0]["number"] = agentsNumber;
	}
	if(numThreads > 0) {
		settings["threads"] = numThreads;
	}
	if(outputDirectory != "") {
		settings["output"]["directory"] = outputDirectory;
	}

	//Si es >=0, entonces se asume que se están haciendo simulaciones
	//
	settings["numExperiment"] = numExperiment;

	// En base a los datos de la sección "input" del archivo
	// de configuración, carga las rutas de los archivos
	// que utiliza el simulador.
	// Debido a que están declarados con rutas relativas al ejecutable,
	// se le agrega la ruta completa.

	std::string map_osrm;
	std::string area_zone_file;
	std::string initial_zones_file;
	std::string reference_zones_file;

	boost::filesystem::path full_path( boost::filesystem::initial_path<boost::filesystem::path>() );
	full_path = boost::filesystem::system_complete( boost::filesystem::path( argv[0] ) );

	g_baseDir = full_path.parent_path().string() + "/" ;

	std::string inputBaseDir = settings["input"]["directory"].get<std::string>() + "/";

	try {
		map_osrm             = g_baseDir + inputBaseDir + settings["input"]["map"].get<std::string>();
		area_zone_file       = g_baseDir + inputBaseDir + settings["input"]["area"].get<std::string>();
		initial_zones_file   = g_baseDir + inputBaseDir + settings["input"]["initial_zones"].get<std::string>();
		reference_zones_file = g_baseDir + inputBaseDir + settings["input"]["reference_zones"].get<std::string>();
	} catch (json::exception &e) {
		std::cerr << "Error in get action from 'input' section in <config.json>:" << std::endl;
		std::cerr << e.what() << std::endl;
		exit(EXIT_FAILURE);
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

	// Carga los datos geográficos de los archivos respectivos
	loadDataFrom(area_zone_file, area_zone);
	loadDataFrom(initial_zones_file, initial_zones);
	loadDataFrom(reference_zones_file, reference_zones);

	//Reset counters
	g_timeExecMakeAgents = 0;
	g_timeExecCal        = 0;
	g_timeExecSim        = 0;

	
	
	Simulator sim(settings, initial_zones, reference_zones, area_zone, map_osrm);

	omp_set_num_threads( settings["threads"].get<uint32_t>() );

	sim.calibrate();
	sim.run();


	return(EXIT_SUCCESS);
}
