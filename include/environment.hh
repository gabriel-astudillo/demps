#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_
#include <glob.hh>
#include <agent.hh>
#include <patchAgent.hh>
#include <router.hh>
#include <zone.hh>

class Environment {

private:
	Router _router;

	LocalCartesian    _projector;
	json              _reference_point;
	std::vector<Zone> _initial_zones;
	std::vector<Zone> _reference_zones;

	std::vector<Agent*> _vAgents;
	std::vector<PatchAgent*> _pAgents; //NEW

	Agent::Neighbors   _neighbors;

	struct grid_s {
		// Largo del cuadrante (en [m])
		uint32_t _quadSize;

		//Dimensiones del mapa
		double _xMin, _xMax, _yMin, _yMax;
		double _mapWidth, _mapHeight;

		//Cantidad de cuadrantes en el eje X e Y
		uint32_t _quadX;
		uint32_t _quadY;
	};

	struct grid_s _grid;

public:
	typedef struct grid_s grid_t;

	Environment(void);
	Environment(const Environment&);
	Environment(std::vector<Agent*>);
	~Environment(void);

	void setRouter(const std::string &_map_osrm);
	Router* getRouter(void);

	void setReferencePoint(const json &_freference_point);
	void setReferenceZones(const json &_freference_zones);
	void setInitialZones(const json &_finitial_zones);
	void setProjector();

	void setGrid(const json &_fmap_zone, uint32_t quadSize);
	grid_t getGrid();
	void showGrid();

	Zone getInitialZone(uint32_t id);
	std::vector<Zone> getInitialZones();
	std::vector<Zone>& getReferenceZones();
	LocalCartesian getProjector();

	void addAgent(Agent* newAgent);
	void addPatchAgent(PatchAgent* newAgent); //NEW

	uint32_t getTotalAgents();
	Agent* getAgent(uint32_t id);
	PatchAgent* getPatchAgent(uint32_t id); //NEW
	std::vector<Agent*>& getAgents();

	void setNeighborsOf(const uint32_t& idAgent,const double& distance);

	void adjustAgentsInitialPosition(const uint32_t& calibrationTime);
	void adjustAgentsRules();

	void updateAgents();
	void updateQuads();
	void updateStats();

	double distance(Agent* a, Agent* b);
	bool isClose(Agent* a, Agent* b, const double& distanceMax);

};

#endif
