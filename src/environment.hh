#pragma once

#include <agent.hh>
#include <patchAgent.hh>
#include <router.hh>
#include <zone.hh>

#include <GeographicLib.hh>
#include <CGAL.hh>
#include <globals.hh>

#include <json.hpp>
using json=nlohmann::json;

class Environment {
public:
	

private:
	mutable std::mutex _mtx; // para debug en metodos paralelos
	
	Router _router;

	LocalCartesian    _projector;
	Point2D           _reference_point; //(longitud, latitud)
	
	std::vector<Zone> _initial_zones;
	std::vector<Zone> _reference_zones;
	std::vector<Zone> _flood_zones;

	std::vector<Agent*> _vAgents;
	std::vector<PatchAgent*> _pAgents;
	std::vector<PatchAgent*> _pAgentsInCity;
	std::vector<PatchAgent*> _pAgentsInStreets;
	
	Zone::NSWEPatchAgentsAllZone_t _NSWEPatchAgentsAllZones;

	Agent::Neighbors   _neighbors;
	
	PatchAgent::monitorGroup _pointsMonitorGroup;
	PatchAgent::monitorGroup _linesMonitorGroup;
	
	struct grid_s {
		// Largo del cuadrante (en [m])
		uint32_t _quadSize;

		//Dimensiones del mapa
		double _xMin, _xMax, _yMin, _yMax;
		double _xSimMin, _xSimMax, _ySimMin, _ySimMax;
		double _mapWidth, _mapHeight;

		//Cantidad de cuadrantes en el eje X e Y
		uint32_t _quadX;
		uint32_t _quadY;
	};
	struct grid_s _grid;
	
	struct floodParams_s{
		bool enable;
		
		std::string direction;
		int32_t    arrivalTime; // en s
		uint32_t    sampleStateInterval; // en s
		double      speedWaterLevel; // en m/s
		double      speedWaterProp;  // en m/s
		double      criticalLevel;   // en m
		double      minSpeedFactor;  // índice [0,1]
		
		bool        imagesEnable;
		std::string imagesDir;
		
		bool        stateEnable;
		std::string stateDir;
	};
	struct floodParams_s _floodParams;
	
	struct densityParams_s{
		bool enable;
		
		double minDensity;
		double maxDensity;
		double minVelocity;
	};
	struct densityParams_s _densityParams;

public:
	typedef struct grid_s          grid_t;
	typedef struct floodParams_s   floodParams_t;
	typedef struct densityParams_s densityParams_t;

	Environment(void);
	Environment(const Environment&);
	Environment(std::vector<Agent*>);
	~Environment(void);

	void setRouter(const std::string& map_osrm);
	Router* getRouter(void);

	
	void setReferenceZones(const json& freference_zones);
	void addReferenceZone(const json& freference_zone_feature);
	
	void setInitialZones(const json& finitial_zones);
	void addInitialZone(const json& finitial_zone_feature);
	
	void addFloodZone(const json& flood_zone_feature);
	
	void addLineMonitorZone(const json& lineMonitor_feature);
	void addPointMonitorZone(const json& pointMonitor_feature);
	
	void setReferencePoint(const json& fmap_zone);
	Point2D getReferencePoint();
	
	void setProjector(const json& fmap_zone);
	LocalCartesian getProjector();

	void setGrid(const json &fmap_zone, uint32_t offset, uint32_t quadSize);
	grid_t getGrid();
	void showGrid();
	
	//
	// Modelo de Inundación
	//
	void setFloodParams(const json& floodParams);
	floodParams_t getFloodParams();
	void assignPatchAgentsToFloodZones();
	void setFloodVelocityOf(Agent* agent);
	
	
	void setNSWEPatchAgentsAllZones();
	Zone::NSWEPatchAgentsAllZone_t getNSWEPatchAgentsAllZones();

	Zone& getInitialZone(uint32_t id);
	std::vector<Zone>& getInitialZones();
	Zone& getReferenceZone(uint32_t id);
	std::vector<Zone>& getReferenceZones();
	Zone& getFloodZone(uint32_t id);
	std::vector<Zone>& getFloodZones();
	void orderFloodZones();
	
	
	json getZonesInfo();
	void getZonesInfo(json& out);

	void addAgent(Agent* newAgent);
	void addPatchAgent(PatchAgent* newAgent);
	void addPatchAgentInCity(PatchAgent* newAgent);
	void addPatchAgentInStreets(PatchAgent* newAgent);
	
	//uint32_t getQuadId(Agent* a);
	uint32_t getQuadId(Point2D position);
	Point2D  getQuadXY(Point2D position);
	std::vector<PatchAgent*>  getPatchAgentsLineMonitor(Point2D pointIni, Point2D pointEnd);
	void getPatchAgentsLineMonitor(Point2D pointIni, Point2D pointEnd, std::vector<PatchAgent*>& patchAgents);
	std::vector<PatchAgent*>  getPatchAgentsPointMonitor(Point2D pointMonitor);
	void getPatchAgentsPointMonitor(Point2D pointMonitor, std::vector<PatchAgent*>& patchAgents);
	
	void addMonitorPatchAgentGroup(PatchAgent* patchAgent, std::string idGroup, PatchAgent::typeMonitor type);
	PatchAgent::monitorGroup getMonitorPatchAgentGroups(PatchAgent::typeMonitor type);

	uint32_t getTotalAgents();
	Agent* getAgent(uint32_t id);
	std::vector<Agent*>& getAgents();
	
	PatchAgent* getPatchAgent(uint32_t id);
	std::vector<PatchAgent*> getPatchAgents();
	std::vector<PatchAgent*> getPatchAgentsInCity();
	std::vector<PatchAgent*> getPatchAgentsInStreets();
	
	void setNeighborsOf(const uint32_t& idAgent,const double& distance);
	
	//
	// Modelo de densidad
	//
	void setDensityParams(const json& densityParams);
	densityParams_t getDensityParams();
	void setDensityOf(Agent* agent);
	void setDensityVelocityOf(Agent* agent);
	
	void setGradientVelocityOf(Agent* agent);
	
	bool setDebrisVelocityOf(Agent* agent);

	void adjustAgentsInitialPosition(const uint32_t& calibrationTime);
	
	void determinatePAgentsInStreets();
	void determinatePAgentsWithDebris(double debrisRatio, int& pAgentsWithDebris);
	void adjustAgentsRules();
	void setSafeZoneAttribAgent(Agent* agent);

	void updateAgents();
	void updateQuads();
	void updateLogsStats();

	double distance(Agent* a, Agent* b);

};

