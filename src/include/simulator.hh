#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_
#include <glob.hh>
#include <zone.hh>
#include <agent.hh>
#include <router.hh>
#include <environment.hh>

class Simulator {
public:
	//static bool        _statsOut;
	//static uint32_t    _statsInterval;


private:
	json _fsettings;
	std::shared_ptr<Environment> _env;

	bool        _statsOut;
	uint32_t    _statsInterval;

	uint32_t    _duration;
	uint32_t    _calibrationTime;
	bool        _saveToDisk;
	uint32_t    _interval;
	uint32_t    _filesimPrecision; //Precision ENU->WSG84
	std::string _filesimSufix;
	std::string _filesim;
	std::string _filesimPath;
	std::string _statsPath;

	void savePositionAgents();
	void saveStats();
	void showTimeExec(void);

public:
	Simulator(void);
	Simulator(const json&,const json&,const json&,const json&, const std::string&);

	void calibrate(void);
	void run(void);

	~Simulator(void);

};
#endif
