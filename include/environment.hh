#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_
#include <glob.hh>
#include <agent.hh>
#include <router.hh>
#include <zone.hh>


class Router;

class Environment{
public:
	std::map<uint32_t,std::list<Point2D>> _routes;
	
	//Router _router;
	//std::vector<Agent> _vAgents;

private:
	Router _router;
	
	LocalCartesian         _projector;
	json                   _reference_point;
	std::vector<Zone>      _initial_zones;
	std::vector<Zone>      _reference_zones;

	std::vector<Agent> _vAgents;
	double _xMin, _xMax, _yMin, _yMax;
	double _mapWidth, _mapHeight;

	// Largo del cuadrante (en [m])
	uint32_t _quadSize;

	//Cantidad de cuadrantes en el eje X e Y
	uint32_t _quadX, _quadY;

public:
	Environment(void);
	Environment(const Environment&);
	Environment(const std::vector<Agent>&);
	~Environment(void);
	
	void setRouter(const std::string &_map_osrm);
	Router* getRouter(void);
	
	void setReferencePoint(const json &_freference_point);
	void setReferenceZones(const json &_freference_zones);
	void setInitialZones(const json &_finitial_zones);
	void setProjector();
	
	void setGrid(const json &_fmap_zone, uint32_t quadSize);
	
	Zone getInitialZone(uint32_t id);
	std::vector<Zone> getInitialZones();
	std::vector<Zone> getReferenceZones();
	LocalCartesian getProjector();

	void addAgents(const std::vector<Agent> &_vAgents);
	uint32_t getTotalAgents();
	Agent* getAgent(uint32_t id);
	std::vector<Agent> getAgents();

	Agent::Neighbors neighbors_of(const Agent&,const double&,const model_t&);

	void adjustAgentsRules(); //Por hacer
	void updateAgents();

};
#endif
