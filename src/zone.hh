#pragma once

#include <agent.hh>

#include <mutex>
#include <json.hpp>
using json=nlohmann::json;

#include <GeographicLib.hh>
#include <CGAL.hh>

struct AvgDym {
	
	double _avg;
	uint32_t _numbers;
	
	AvgDym(): _numbers(0){}
	
	double operator() (double n){	
		_numbers++;
		
		_avg = (double)(_numbers-1)/(double)_numbers * _avg + (double)n/(double)_numbers;
		return(_avg);
	}

};

class Environment;


class Zone {
public:
	static std::shared_ptr<Environment> _myEnv;
	
	typedef std::array<uint32_t,4> NSWEPatchAgentsInZone_t;
	typedef std::array<uint32_t,4> NSWEPatchAgentsAllZone_t;
	
private:
	mutable std::mutex _mtx;
	
	std::string          _nameID;
	std::string          _zoneType;
	double               _maxLevelFlood;
	int32_t			     _order;
	
	double               _area;
	LocalCartesian       _projector;
	Polygon2D            _polygon;
	CDT                  _cdt;

	std::set<uint32_t> _agentsInZone;
	std::set<uint32_t> _agentsAssignedInZone;
	std::set<uint32_t> _patchAgentsInZone;
	double             _agentsDensity;
	
	NSWEPatchAgentsInZone_t _NSWEAgentsInZone;
	
	Point2D _centroid; // (x,y)
	Point2D _xyMin;
	Point2D _xyMax;
	AvgDym _avgCentroidX, _avgCentroidY;
	double _shapeForm; //factor de forma de la zona
	Vector2D _rEquiv;  //semilados del area rectangular equivalente (rw,rh)

public:
	Zone(void);
	
	Zone(const json&);
	Zone(const Zone&);
	~Zone(void);

	Zone& operator=(const Zone&);

	bool pointIsInside(const Point2D& testPoint, const double& bias);
	
	void addAgent(const uint32_t& idAgent);
	void deleteAgent(const uint32_t& idAgent);
	const std::set<uint32_t>& agentsInZone(void);
	
	void addAgentAssigned(const uint32_t& idAgent);
	void deleteAgentAssigned(const uint32_t& idAgent);
	const std::set<uint32_t>& agentsAssignedInZone(void);
	
	void updateAgentsDensity(void);
	uint32_t getTotalAgents(void);
	double getAgentsDensity(void);
	
	uint32_t getTotalAgentsAssigned(void);
	
	std::string getNameID(void) const;
	std::string getZoneType() const;
	double      getMaxLevelFlood() const;
	int32_t    getOrder() const;
	
	Point2D generate(void);
	
	double getArea();
	Point2D getCentroid();
	Point2D getCentroidWGS84();
	
	Point2D getXYmin();
	Point2D getXYminWGS84();
	Point2D getXYmax();
	Point2D getXYmaxWGS84();	
	double  getshapeForm();
	Vector2D getRequiv();
	
	
	//void assignPatchAgents();
	void addPatchAgent(uint32_t patchID);
	void deletePatchAgent(const uint32_t& patchID);
	const std::set<uint32_t>& patchAgentsInZone(void);
	void setNSWEPatchAgentsInZone(void);
	const NSWEPatchAgentsInZone_t& getNSWEPatchAgentsInZone(void);

private:
	void mark_domains(CDT& cdt);
	void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border );
};


/////////////////////////////////////////////////////////////////////////
// zoneSimilarity declaration
//
struct zonesSimilarity{
private:
	static void sortZones(Zone&, Zone &);
public:
	static double jaccardIndex(Zone , Zone);
};


