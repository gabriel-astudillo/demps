#pragma once

#include <mutex>
#include <json.hpp>
using json=nlohmann::json;

#include <globals.hh>


class Environment;
class Zone;

class PatchAgent {

public:

	typedef std::vector<PatchAgent*> Neighbors;
	typedef std::vector<PatchAgent*> Group;
	typedef std::vector<uint32_t> idPatchNeighbors;
	typedef std::map<std::string, PatchAgent::Group> monitorGroup;
	
	typedef enum {pointMonitor, lineMonitor} typeMonitor;
		
	static std::shared_ptr<Environment> _myEnv;


private:
	mutable std::mutex _mtx;
	uint32_t _id;
	//uint32_t _quad;

	std::vector<uint32_t> _agentsInPatch;
	std::vector<uint32_t> _neighborsAgents; //NEW
	
	Zone* _myZone;
	
	// Características geométricas del cuadrante del patchAgent.
	struct quad_s{
		// Coordenadas (x,y) del vértice inferior izquierdo.
		double x0;
		double y0;
		
		// Coordenadas (x,y) del vértice superior derecho.
		double x1, y1;
		
		// Coordenadas (x,y) del centro.
		double xc, yc;
		
		// Coordenadas (lat,lon) del centro.
		double lat, lon;
		
		// ancho y alto.
		double width, height;
		
		// identificadores numericos del cuadrante
		uint32_t idX, idY;
	};
	struct quad_s _myQuad;
	
	// Todos los patchAgents no monitorean por omisión
	bool _isEvacMonitor = false;
	//double _gradient = 0.0;
	
	// elevacion -1 implica que no se le ha calculado al patch
	double  _haveElevation = false;
	int32_t _elevation = -1;
	
	// Todos los patchAgents no son inundables por omisión
	bool _isFloodable = false;
	
	// Nivel de inundación del patch agent, en metros
	double _levelFlood; 
	
	// Nivel máximo de inundación del patch agent, en metros
	double _maxLevelFlood;
	
	// bandera que permite actualizar el nivel de agua
	// sólo el primer agente de un cuadrante puede invocar 
	// la función 'updateLevelFlood()'
	//bool _isFloodLevelUpdatable = true;
	
	// Probabilidad que tenga escombros
	double _probDebris = 0.0;
	bool   _isDebrisFree = true;
	
	bool _isInCity = false;
	

public:
	typedef struct quad_s quad_t;
	
	PatchAgent(void);
	PatchAgent(const uint32_t&);

	~PatchAgent(void);

	uint32_t getId();
	void addAgent(const uint32_t &idAgent);
	void delAgent(const uint32_t &idAgent);
	std::vector<uint32_t>& getAgents();
	
	std::vector<uint32_t>& getNeighborsAgents();
	idPatchNeighbors findPatchNeighbors4();
	idPatchNeighbors findPatchNeighbors();
	
	quad_t getQuadInfo();
	
	void evacMonitor(bool m);
	bool isEvacMonitor();
	
	//void setGradient(double g);
	//double getGradient();
	
	bool haveElevation();
	void haveElevation(bool h);
	int32_t getElevation();
	void    setElevation(int32_t elevation);
	
	bool isInCity();
	void isInCity(bool c);
	void   setProbDebris(double p);
	double getProbDebris();
	bool isDebrisFree();
	void isDebrisFree(bool d);
	
	void setZone(Zone* z);
	Zone* getZone();
	
	void isFloodable(bool m);
	bool isFloodable();
	
	void updateLevelFlood();
	double getLevelFlood();
	
	//bool isFloodLevelUpdatable();
	//void isFloodLevelUpdatable(bool i);
	void setMaxLevelFlood(double m);
	double getMaxLevelFlood();
};
