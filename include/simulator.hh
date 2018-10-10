#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_
#include <glob.hh>
#include <zone.hh>
#include <agent.hh>
#include <router.hh>
#include <environment.hh>

class Simulator{

private:
	static const std::hash<std::string> _hash;
	json                                _fsettings;

	static std::shared_ptr<Environment> _env;
	
	uint32_t    _duration;
	uint32_t    _calibrationTime;
	bool        _saveToDisk;
	uint32_t    _interval;
	std::string _filesimPrefix;
	std::string _filesimSufix;
	std::string _filesimPath;

	void save(const uint32_t&);

public:
	Simulator(void);
	Simulator(const json&,const json&,const json&,const json&,const json&, const std::string&);

	void calibrate(void);
	void run(void);

	~Simulator(void);
	void showTimeExec(void);
};
#endif
