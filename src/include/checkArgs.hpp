#ifndef _CHECKARGS_H_
#define _CHECKARGS_H_

#include <unistd.h>

extern char *optarg;
extern int optind, opterr, optopt;

class checkArgs {
private:
	// 1) Modificar esta sección
	const std::string optString = "c:d:n:t:o:e:h";

	const std::string opciones = "-c file [-d tiempo] [-n número] [-t número] [-o directorio] [-e nro.experimento] [-h]";

	const std::string descripcion  = "Descripción:\n"
	                                 "\t-c   Archivo de configuración JSON\n"
	                                 "\t-d   tiempo de simulación. En segundos.\n"
	                                 "\t-n   Cantidad de agentes.\n"
	                                 "\t-t   Cantidad de threads.\n"
									 "\t-o   Directorio de salida.\n"
									 "\t-e   Numero de experimento.\n"
	                                 "\t-h   Muestra esta salida y termina.\n";

	typedef struct args_t {
		std::string fileConfig;
		uint32_t duration;
		uint32_t agentsNumber;
		uint32_t numThreads;
		std::string outputDirectory;
		int32_t numExperiment;
	} args_t;

	// 2) Modificar constructor
	// 3) Modificar ciclo "getopt" en método checkArgs::getArgs()
	// 4) Recuerde que para compilar nuevamente, y por tratarse
	//    de un archivo header, debe hacer primero un make clean

	args_t  parametros;

	int argc;
	char **argv;


public:
	checkArgs(int _argc, char **_argv);
	~checkArgs();
	args_t getArgs();

private:
	void printUsage();


};

checkArgs::checkArgs(int _argc, char **_argv)
{
	parametros.fileConfig   = "";
	parametros.duration     = 0;
	parametros.agentsNumber = 0;
	parametros.numThreads   = 0;
	parametros.outputDirectory = "";
	parametros.numExperiment = -1;


	argc = _argc;
	argv = _argv;

	getArgs();

}

checkArgs::~checkArgs()
{

}

checkArgs::args_t checkArgs::getArgs()
{
	int opcion;

	while ((opcion = getopt (argc, argv, optString.c_str())) != -1) {
		switch (opcion) {
		case 'c':
			parametros.fileConfig = optarg;
			break;
		case 'd':
			parametros.duration = atoi(optarg);
			break;
		case 'n':
			parametros.agentsNumber = atoi(optarg);
			break;
		case 't':
			parametros.numThreads = atoi(optarg);
			break;
		case 'o':
			parametros.outputDirectory = optarg;
			break;
		case 'e':
			parametros.numExperiment=atoi(optarg);
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

	return(parametros);
}

void checkArgs::printUsage()
{
	std::cout << "Uso: " <<
	          argv[0] << " " << opciones << " " << descripcion << std::endl;
}



#endif
