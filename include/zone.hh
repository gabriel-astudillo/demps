#ifndef _ZONE_H_
#define _ZONE_H_
#include <glob.hh>

class Zone {
private:
	LocalCartesian       _projector;
	Polygon2D            _polygon;
	CDT                  _cdt;

public:
	Zone(void);
	Zone(const json&,const json&);
	Zone(const Zone&);
	~Zone(void);

	Zone& operator=(const Zone&);

	Point2D generate(void);
	
private:
	void mark_domains(CDT& cdt);
	void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border );
};
#endif
