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
	double             _agentsDensity;

public:
	Zone(void);
	Zone(const json&,const json&);
	Zone(const Zone&);
	~Zone(void);

	Zone& operator=(const Zone&);

	bool pointIsInside(const Point2D& testPoint);
	void addAgent(const uint32_t& idAgent);
	void deleteAgent(const uint32_t& idAgent);
	void updateAgentsDensity(void);
	uint32_t getTotalAgents(void);
	double getAgentsDensity(void);
	std::string getNameID(void);
	Point2D generate(void);

private:
	void mark_domains(CDT& cdt);
	void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border );
};

#endif
