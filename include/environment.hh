#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_
#include <glob.hh>
#include <agent.hh>

class Environment{
public:
    //typedef KDTree::KDTree<2,Agent> kdtree;

private:
    //std::shared_ptr<kdtree> _tree;
	
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
    Agent::Neighbors neighbors_of(const Agent&,const double&,const model_t&);
    void update(const std::vector<Agent>&);
};
#endif
