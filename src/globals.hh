#pragma once
#include <string>
#include <vector>
#include <sstream>

#include <restclient-cpp/restclient.h>
enum CURLerrorCode {COULDNT_CONNECT = 7};


enum model_t {Residents=0, Visitors_I=1, Visitors_II=2};
extern std::map<std::string, model_t> model_map;

/*
//               Ts                   Ak                   N(k,s)    %N     min(k) max(k) meanDist
extern std::map< uint32_t,  std::map< uint32_t, std::tuple<uint32_t, float, float, float, float> > > g_sdataDistSafeZoneByAgeGroup;
//               Ts                   Zn                      E(n,s)   %E(n,s) tevac_min  tevac_max  tevac_mean
extern std::map< uint32_t,  std::map< std::string, std::tuple<uint32_t, float, float    , float    , float      > > > g_sdataEvacTimeByZone;
*/

////////////////////////////////////////////////////////////////////////////////////////////////
// versión con los índices en std::string para permitir la conversión a JSON en forma automática
//
//                         Ts                      Ak                      N(k,s)    %N     min(k) max(k) meanDist
extern std::map< std::string,  std::map< std::string, std::tuple<uint32_t, float, float, float, float> > > g_sdataDistSafeZoneByAgeGroup;
//                         Ts                      Zn                      E(n,s)   %E(n,s) tevac_min  tevac_max  tevac_mean
extern std::map< std::string,  std::map< std::string, std::tuple<uint32_t, float, float    , float    , float      > > > g_sdataEvacTimeByZone;


namespace global{
	/*
	 * Variables globales
	 *
	 * 		global::currTimeSim
	 *
	 */
	extern uint32_t    currTimeSim; 
	

	/*
	 * Variables de salidas del simulador
	 *
	 *		global::agentsMem
	 *		global::totalAgentsInSim
	 *
	 *		global::simOutputs.timeExec.makeAgents
	 *		global::simOutputs.timeExec.calibration
	 *		global::simOutputs.timeExec.simulation
	 *
	 *		global::simOutputs.logs.zonesDensity
	 *		global::simOutputs.logs.usePhone
	 *		global::simOutputs.logs.velocity
	 *		global::simOutputs.logs.SIRpanic
	 *		global::simOutputs.logs.deceasedAgents
	 *		global::simOutputs.logs.stepDelay
	 *		global::simOutputs.logs.evacTime
	 */
	struct SimOutputs_s {
		uint32_t    agentsMem;        
		uint32_t    totalAgentsInSim; 
		struct TimeExec_s {
			uint32_t makeAgents;  
			uint32_t calibration; 
			uint32_t simulation;  
		} timeExec;  

		struct Logs_s {
			std::vector<std::string> zonesDensity; 
			std::vector<uint32_t>    usePhone;     
			std::vector<std::string> velocity;     
			std::vector<std::string> SIRpanic;     
			std::vector<std::string> deceasedAgents;
			std::vector<std::string> stepDelay;
			std::vector<double>      evacTime;
		} logs;
	};
	extern SimOutputs_s simOutputs;

	/*
	 * Opciones de ejecución del simulador
	 *
	 *		global::execOptions.showProgressBar
	 *		global::execOptions.agentsOut
	 */
	struct ExecOptions_s {      
		bool showProgressBar; 
		bool agentsOut;
		bool logToCOUT = true;
	};
	extern ExecOptions_s execOptions;

	/*
	 * Parámetros del simulador
	 *
	 *		global::params.deltaT
	 *		global::params.closeEnough
	 *		global::params.randomWalkRadius
	 *		global::params.attractionRadius
	 *		global::params.configDir
	 *		global::params.configFile
	 *
	 *      global::params.sampling.compareWithOthersSims
	 *      global::params.sampling.interval
	 *      global::params.sampling.level
	 *      global::params.sampling.saveSimInDB
	 *
	 *		global::params.offsetMap
	 *		global::params.animationDir
	 *		global::params.animationFile
	 *
	 *		global::params.modelsEnable.panic
	 *		global::params.modelsEnable.elevation
	 *		global::params.modelsEnable.debris
	 *		global::params.modelsEnable.flood
	 *
	 *		global::params.watchDog.initialWaitTime
	 *		global::params.watchDog.deltaTime
	 *		global::params.watchDog.thresTime
	 *		global::params.watchDog.pidFile
	 *
	 *		global::params.snitchServer.URL
	 *		global::params.snitchServer.seekRadius
	 *		global::params.snitchServer.xsteps
	 *		global::params.snitchServer.cutOff
	 *		global::params.snitchServer.seekRadiusHMap
	 *		global::params.snitchServer.cutoffHMap
	 *
	 *		global::params.elevationServer.URL
	 *		global::params.elevationServer.coorTest
	 */
	struct Params_s {
		float deltaT;              
		float closeEnough;         
		float randomWalkwayRadius; 
		float attractionRadius;  
		std::string configDempsDir = "/usr/local/etc/demps/";
		std::string configDir = "conf.d/";
		std::string configFile = "demps.conf";

		struct Sampling_s {
			bool     compareWithOthersSims = false;
			uint32_t interval;
			double   level = 0.5;
			bool     saveSimInDB   = false;
		} sampling;

		uint32_t offsetMap = 500; // meters
		std::string animationDir  = configDempsDir + "animation/";
		std::string animationFile = "animation.html";

		struct ModelsEnable_s {
			bool panic;     
			bool elevation; 
			bool debris;    
			bool flood;     
		} modelsEnable;

		struct WathDog_s {
			uint32_t initialWaitTime = 60;  //segundos
			uint32_t deltaTime       = 60;  //segundos
			uint32_t thresTime       = 30; //segundos
			std::string pidFile      = "demps.pid";
		} watchDog;

		struct SnitchServer_s {
			std::string URL         = "http://127.0.0.1:6502/v1/snitch/api";
			double   seekRadius     = 0.5; 
			uint32_t xsteps         = 10;
			uint32_t cutOff         = 10;
			double   seekRadiusHMap = 5;
			uint32_t cutoffHMap     = 10;
		} snitchServer;

		struct ElevationServer_s {
			std::string URL      = "http://127.0.0.1:64000";
			std::string coorTest = "-33.144995,-71.568655";
		} elevationServer;
	};
    extern Params_s params;
}