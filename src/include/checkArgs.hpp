#pragma once

#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>

extern char *optarg;
extern int optind, opterr, optopt;

class CheckArgs {
private:
	// 1) Modificar esta sección
	//const std::string optString = "c:d:D:e:f:p:P:b:B:v:V:n:N:t:T:o:s:hE";
	//const std::string optString = "c:d:D:e:f:p:P:b:B:v:n:N:t:T:o:s:hE";
	const std::string optString = "c:d:D:e:f:p:P:b:B:v:n:N:t:T:o:s:hm";
	
	const struct option optStringLong[19] = {
		{"makeconfig"  , no_argument      , nullptr, 'm'},
		{"config"      , required_argument, nullptr, 'c'},
		{"densitymodel", required_argument, nullptr, 'd'},
		{"description" , required_argument, nullptr, 'D'},
		{"experiment"  , required_argument, nullptr, 'e'},
		{"floodmodel"  , required_argument, nullptr, 'f'},
		{"panicmodel"  , required_argument, nullptr, 'p'},
		{"emotionthreshold", required_argument, nullptr, 'P'},
		{"debrismodel" , required_argument, nullptr, 'b'},
		{"debrisratio" , required_argument, nullptr, 'B'},
		{"elevationmodel", required_argument, nullptr, 'v'},
		//{"elevationfile", required_argument, nullptr, 'V'},
		{"residents"   , required_argument, nullptr, 'n'},
		{"visitors"    , required_argument, nullptr, 'N'},
		{"threads"     , required_argument, nullptr, 't'},
		{"timesim"     , required_argument, nullptr, 'T'},
		{"outdir"      , required_argument, nullptr, 'o'},
		{"sampliglevel", required_argument, nullptr, 's'},
		//{"patchcoords" , no_argument, nullptr, 'E'},
		{"help", no_argument, nullptr, 'h'},
		{nullptr, no_argument, nullptr, 0}
	};

	const std::string arguments = "-c|--config config_file [options] [-h|--help]";
	
	const std::string description = "DESCRIPTION:\n" "\tDEMPS simulator.";

	const std::string options  = "OPTIONS:\n"
	                                 "\t-c, --config            config file (JSON).\n"
									 "\t-t, --threads           threads to use.\n"
	                                 "\t-T, --timesim           total simulation time.\n"
									 "\t-s, --samplinglevel     sampling level 0..1\n"
									 "\t-o, --outdir            output directory (overwrite config file).\n"
									 "\t-n, --residents         number of residents agents.\n"
									 "\t-N, --visitors          number of visitors agents.\n"
									 "\t-D, --description       simulation description, e.g: --description \"test city\".\n"										 
									 "\t-d, --densitymodel      enable density model (1:true, 0:false, -1:config).\n"
									 "\t-p, --panicmodel        enable panic model (1:true, 0:false, -1:config).\n"
									 "\t-P, --emotionthreshold  emotion threshlod for enter to panic state.\n"
									 "\t-f, --floodmodel        enable flood model (1:true, 0:false, -1:config).\n"			 
									 "\t-b, --debrismodel       enable debris model (1:true, 0:false, -1:config).\n"
									 "\t-B, --debrisratio       ratio of patchs with debris (1..100)%.\n"
									 "\t-v, --elevationmodel    enable elevation model (1:true, 0:false, -1:config).\n"
									 //"\t-V, --elevationfile     file with data elevation. Default: 'elevationPatchData.txt'.\n"
									 //"\t-E, --patchcoords       save in file 'elevationPatch-<city>.txt the coords (lat,lon) of each patch and end.\n"
									 "\t-e, --experiment        experiment number.\n"							 
	                                 "\t-h, --help              show this help and end.\n"
									 "\t-m, --makeconfig        create default config file.\n";
 
	typedef struct args_t {
		std::string fileConfig;
		bool        makeConfig;
		uint32_t duration;
		int32_t floodModel;
		int32_t panicModel;
		int32_t emotionThreshold;
		int32_t densityModel;
		int32_t debrisModel;
		int32_t debrisRatio;
		int32_t elevationModel;
		//std::string elevationFile;
		std::string description;
		uint32_t agentsResidentsNumber;
		uint32_t agentsVisitorsNumber;
		uint32_t numThreads;
		std::string outputDirectory;
		int32_t numExperiment;
		double samplingLevel;
		//bool   patchCoords;
	} args_t;

	// 2) Modificar constructor
	// 3) Modificar ciclo "getopt" en método checkArgs::getArgs()
	// 4) Recuerde que para compilar nuevamente, y por tratarse
	//    de un archivo header, debe hacer primero un make clean

	args_t  parametros;

	int argc;
	char **argv;


public:
	CheckArgs(int _argc, char **_argv);
	~CheckArgs();
	void   loadArgs();
	args_t getArgs();

private:
	void printUsage();

};

CheckArgs::CheckArgs(int _argc, char **_argv)
{
	parametros.fileConfig   = "";
	parametros.makeConfig   = false;
	parametros.duration     = 0;
	parametros.floodModel   = -1;
	parametros.panicModel   = -1;
	parametros.emotionThreshold = 0;
	parametros.densityModel = -1;
	parametros.debrisModel  = -1;
	parametros.debrisRatio  = 1;
	parametros.elevationModel = -1;
	//parametros.elevationFile = "elevationPatchData.txt";
	parametros.description  = "";
	parametros.agentsResidentsNumber = 0;
	parametros.agentsVisitorsNumber  = 0;
	parametros.numThreads   = 0;
	parametros.outputDirectory = "";
	parametros.numExperiment = -1;
	parametros.samplingLevel = -1;
	//parametros.patchCoords = false;


	argc = _argc;
	argv = _argv;

	loadArgs();

}

CheckArgs::~CheckArgs()
{

}

void CheckArgs::loadArgs()
{
	int opcion;
	int option_index;
	//while ((opcion = getopt (argc, argv, optString.c_str())) != -1) {
	while ((opcion = getopt_long (argc, argv, optString.c_str(),  optStringLong, &option_index)) != -1) {
		switch (opcion) {
		case 'c':
			parametros.fileConfig = optarg;
			break;
		case 'T':
			parametros.duration = std::atoi(optarg);
			break;
		case 'f':
			parametros.floodModel = std::atoi(optarg);
			break;
		case 'p':
			parametros.panicModel = std::atoi(optarg);
			break;
		case 'P':
			parametros.emotionThreshold = std::atoi(optarg);
			break;
		case 'b':
			parametros.debrisModel = std::atoi(optarg);
			break;
		case 'B':
			parametros.debrisRatio = std::atoi(optarg);
			break;
		case 'd':
			parametros.densityModel = std::atoi(optarg);
			break;
		case 'v':
			parametros.elevationModel = std::atoi(optarg);
			break;
		//case 'V':
		//	parametros.elevationFile = optarg;
		//	break;
		case 'n':
			parametros.agentsResidentsNumber = std::atoi(optarg);
			break;
		case 'N':
			parametros.agentsVisitorsNumber  = std::atoi(optarg);
			break;
		case 'D':
			parametros.description  = optarg;
			break;
		case 't':
			parametros.numThreads = std::atoi(optarg);
			break;
		case 'o':
			parametros.outputDirectory = optarg;
			break;
		case 'e':
			parametros.numExperiment=std::atoi(optarg);
			break;
		case 's':
			parametros.samplingLevel=std::atof(optarg);
			break;
		case 'm':
			parametros.makeConfig = true;
			break;
		case 'h':
		default:
			printUsage();
			exit(EXIT_FAILURE);
		}
	}

	if ( parametros.fileConfig == "" && parametros.makeConfig == false) {
		printUsage();
		exit(EXIT_FAILURE);
	}
}

CheckArgs::args_t CheckArgs::getArgs()
{
	return(parametros);
}

void CheckArgs::printUsage()
{
	std::cout << "Uso: " <<
	          argv[0] << " " << arguments << "\n" << description << "\n" << options << std::endl;
}

