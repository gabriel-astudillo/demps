#ifndef _CHECKARGS_H_
#define _CHECKARGS_H_

#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>

extern char *optarg;
extern int optind, opterr, optopt;

class CheckArgs {
private:
	// 1) Modificar esta sección
	const std::string optString = "c:d:D:e:f:p:P:b:B:v:V:n:N:t:T:o:s:hE";
	
	const struct option optStringLong[20] = {
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
		{"elevationfile", required_argument, nullptr, 'V'},
		{"residents"   , required_argument, nullptr, 'n'},
		{"visitors"    , required_argument, nullptr, 'N'},
		{"threads"     , required_argument, nullptr, 't'},
		{"timesim"     , required_argument, nullptr, 'T'},
		{"outdir"      , required_argument, nullptr, 'o'},
		{"sampliglevel", required_argument, nullptr, 's'},
		{"patchcoords" , no_argument, nullptr, 'E'},
		{"help", no_argument, nullptr, 'h'},
		{nullptr, no_argument, nullptr, 0}
	};

	const std::string opciones = "-c file [-d tiempo] [-f 0|1] [-n ag.residentes]  [-N ag.visitantes] [-D string] [-t nro.threads] [-o directorio] [-e nro.experimento] [-E][-h]";

	const std::string descripcion  = "Descripción:\n"
	                                 "\t-c   Archivo de configuración JSON\n"
	                                 "\t-T   tiempo de simulación. En segundos.\n"
									 "\t-d   1: habilita modelo de densidad. 0: inhabilita modelo de densidad. -1: directiva archivo configuración.\n"
									 "\t-f   1: habilita modelo de inundación. 0: inhabilita modelo de inundación. -1: directiva archivo configuración.\n"
									 "\t-p   >0: habilita modelo de pánico. 0: inhabilita modelo de pánico. -1: directiva archivo configuración.\n"
									 "\t-P   Indica el porcentaje de emoción para que se considere infectado.\n"
									 "\t-b   1: habilita modelo de escombros. 0: inhabilita modelo de escombros. -1: directiva archivo configuración"
									 "\t-B   Proporción de patch con escombros.\n"
									 "\t-v   1: habilita modelo de elevación. 0: inhabilita modelo de elevación. -1: directiva archivo configuración"
									 "\t-V   Archivo de datos de elevación. Por omision, 'elevationPatchData.txt'."
									 "\t-n   Cantidad de agentes residentes.\n"
									 "\t-N   Cantidad de agentes visitantes.\n"
									 "\t-D   Descripción de la simulación.\n"
	                                 "\t-t   Cantidad de threads.\n"
									 "\t-o   Directorio de salida.\n"
									 "\t-e   Numero de experimento.\n"
									 "\t-E   Guarda en 'elevationPatch.txt' las coordenadas (lat,lon) de cada patch y termina.\n"
	                                 "\t-h   Muestra esta salida y termina.\n";

	typedef struct args_t {
		std::string fileConfig;
		uint32_t duration;
		int32_t floodModel;
		int32_t panicModel;
		int32_t emotionThreshold;
		int32_t densityModel;
		int32_t debrisModel;
		int32_t debrisRatio;
		int32_t elevationModel;
		std::string elevationFile;
		std::string description;
		uint32_t agentsResidentsNumber;
		uint32_t agentsVisitorsNumber;
		uint32_t numThreads;
		std::string outputDirectory;
		int32_t numExperiment;
		double samplingLevel;
		bool   patchCoords;
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
	parametros.duration     = 0;
	parametros.floodModel   = -1;
	parametros.panicModel   = -1;
	parametros.emotionThreshold = 0;
	parametros.densityModel = -1;
	parametros.debrisModel  = -1;
	parametros.debrisRatio  = 1;
	parametros.elevationModel = -1;
	parametros.elevationFile = "elevationPatchData.txt";
	parametros.description  = "";
	parametros.agentsResidentsNumber = 0;
	parametros.agentsVisitorsNumber  = 0;
	parametros.numThreads   = 0;
	parametros.outputDirectory = "";
	parametros.numExperiment = -1;
	parametros.samplingLevel = -1;
	parametros.patchCoords = false;


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
		case 'V':
			parametros.elevationFile = optarg;
			break;
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
		case 'E':
			parametros.patchCoords=true;
			break;
		case 'h':
		default:
			printUsage();
			exit(EXIT_FAILURE);
		}
	}

	if ( parametros.fileConfig == "" ) {
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
	          argv[0] << " " << opciones << " " << descripcion << std::endl;
}



#endif
