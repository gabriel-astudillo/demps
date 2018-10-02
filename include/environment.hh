#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_
#include <glob.hh>
#include <agent.hh>
#include <router.hh>


class Router;

class Environment{
public:
    std::map<uint32_t,std::list<Point2D>> _routes;

private:
	Router _router;
	
	std::vector<Agent> _vAgents;
	double _xMin, _xMax, _yMin, _yMax;
	double _mapWidth, _mapHeight;
	
	// Largo del cuadrante (en [m])
	uint32_t _quadSize;
	
	//Cantidad de cuadrantes en el eje X e Y
	uint32_t _quadX, _quadY;

public:
    Environment(void);
	Environment(double xMin,double xMax, double yMin, double yMax, uint32_t quadSize);

	Environment(const Environment&);
    Environment(const std::vector<Agent>&);
    ~Environment(void);
    Environment& operator=(const Environment&);

	void addAgents(const std::vector<Agent> &_vAgents);
	uint32_t getTotalAgents();
	Agent* getAgent(uint32_t id);
	std::vector<Agent> getAgents();
	
    Agent::Neighbors neighbors_of(const Agent&,const double&,const model_t&);
    
	void updateAgents();
	
	void setRouter(const json &_freference_point,const std::string &_map_osrm);
	Router getRouter(void);
};
#endif
