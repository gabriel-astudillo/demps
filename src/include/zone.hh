#ifndef _ZONE_H_
#define _ZONE_H_
#include <glob.hh>

class Zone {
private:
	std::string          _nameID;
	double               _area;
	LocalCartesian       _projector;
	Polygon2D            _polygon;
	CDT                  _cdt;

	std::set<uint32_t> _agentsInZone;
	std::set<uint32_t> _agentsAssignedInZone;
	double             _agentsDensity;

public:
	Zone(void);
	Zone(const json&,const json&);
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
	std::string getNameID(void);
	Point2D generate(void);
	
	double getArea();

private:
	void mark_domains(CDT& cdt);
	void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border );
};

#endif
