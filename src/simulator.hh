#pragma once

#include <zone.hh>
#include <agent.hh>
#include <router.hh>
#include <environment.hh>
#include <utils.hh>
#include <DTW.hh>
#include <Timer.hpp>
#include <filesystem>
#include <PGM.hpp>
#include <globals.hh>
#include <LogHandler.hpp>
#include <progressBar.hpp>

#include <csignal>
#include <thread>
#include <sys/resource.h>

#include <sys/types.h>
#include <unistd.h>

#include <json.hpp>
using json=nlohmann::json;


class Simulator {
public:

private:
	utils::uuidSimulation_t _uuidSim;
	
	json _fsettings;
	std::shared_ptr<Environment> _env;

	int32_t     _numExperiment;

	uint32_t    _duration;
	uint32_t    _calibrationTime;
	bool        _agentsOut;
	uint32_t    _interval;
	
	uint32_t    _filesimPrecision; //Precision ENU->WSG84
	std::string _filesimSufix;
	std::string _agentsPath;
	
	bool        _heatMapOut;
	uint32_t    _heatMapSize;
	uint32_t    _heatMapInterval;
	std::string _heatMapPath;
	
	std::string _debrisFilePath;
	
	bool        _statsOut;
	uint32_t    _statsInterval;
	std::string _statsPath;

	std::string _animPath;
	
	//Intervalo para muestrear la simulación
	//uint32_t    _samplingInterval;
	//double      _samplingLevel = 0.5;
	//bool        _saveSimInDB   = false;
	std::mutex  _mtx, _mtxEx;
	
	
	std::string _animConfig;
	std::string _pidFilePath;

	void savePositionAgents();
	std::string saveStatePatchAgents();
	void        saveStateFlood();
	void        saveStateDebris();
	void        saveImgFlood();
	//void saveStats();
	void executionSummary();
	void makeStats();
	
	void samplingSim();

	
	std::string getNameSuffix();

public:
	Simulator(void);
	Simulator(const json&, const json&, const std::string&);

	void calibrate(void);
	void run(void);

	~Simulator(void);
	
private:
	static std::mutex _execForMTX;
	static bool _simInExec;// = true;
	
	uint32_t getMaxMemory();
public:
	void watchDog(uint32_t initialWaitTime,  uint32_t deltaTime,  uint32_t thresTime, std::string dirTodDelete);

};
