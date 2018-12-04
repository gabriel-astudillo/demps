#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_
#include <glob.hh>
#include <zone.hh>
#include <agent.hh>
#include <router.hh>
#include <environment.hh>

class Simulator{
public:
	static bool        _statsOut;
	static uint32_t    _statsInterval;


private:
	//static const std::hash<std::string> _hash;
	json                                _fsettings;

	static std::shared_ptr<Environment> _env;
	
	uint32_t    _duration;
	uint32_t    _calibrationTime;
	bool        _saveToDisk;
	uint32_t    _interval;
	std::string _filesimPrefix;
	std::string _filesimSufix;
	std::string _filesimPath;
	std::string _statsPath;

	void save();
	void stats();

public:
	Simulator(void);
	Simulator(const json&,const json&,const json&,const json&,const json&, const std::string&);

	void calibrate(void);
	void run(void);

	~Simulator(void);
	void showTimeExec(void);
};
#endif
