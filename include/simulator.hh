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

	Router                              _router;
	Environment                          env;
	LocalCartesian                      _projector;
	std::vector<GeoCoords>              _reference_points;
	std::vector<Zone>                   _initial_zones;
	std::vector<Zone>                   _reference_zones;
	std::vector<Agent>                  _agents;

	std::map<uint32_t,std::list<Point2D>> _routes;

	void run(const uint32_t&);
	void save(const uint32_t&);

public:
	Simulator(void);
	Simulator(const json&,const json&,const json&,const json&,const std::string&);

	void run(void);
	void calibrate(void);

	~Simulator(void);
};
#endif
