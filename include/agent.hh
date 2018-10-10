#ifndef _AGENT_HH_
#define _AGENT_HH_
#include <glob.hh>

class Environment;

class Agent{
public:
	typedef std::vector<Agent> Neighbors;
	
	std::list<Point2D> _route; // Primer intento. Debe ser un atributo privado.
	
private:
	uint32_t _id;
	double   _min_speed;
	double   _max_speed;

	Point2D  _position;
	uint32_t _quad;
	Vector2D _direction;

	model_t  _model;
	
	static std::shared_ptr<Environment> _myEnv;
	

public:
	Agent(void);
	Agent(const Agent&);
	Agent(const uint32_t&,const Point2D&,const double&,const double&,const model_t&);
 
	~Agent(void);
	Agent& operator=(const Agent&);

	Point2D  position(void) const;
	void     showPosition();
	uint32_t determineQuad();
	void     setQuad();
	uint32_t getQuad();
	Vector2D direction(void) const;

	uint32_t id(void) const;
	model_t model(void) const;
	
	void setEnvironment(std::shared_ptr<Environment> env);
	
	void update();

	void shortestPath();
	void followPath();
	void randomWalkway();
	void followTheCrowd(const Neighbors&);

	double distance(Agent const &_agent) const{
		return(sqrt(CGAL::squared_distance(this->_position,_agent._position)));
	}
};
#endif
