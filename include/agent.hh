#ifndef _AGENT_HH_
#define _AGENT_HH_
#include <glob.hh>

class Environment;

class Agent{
public:
	typedef double value_type;
	typedef std::vector<Agent> Neighbors;
	
private:
	uint32_t _id;
	double   _min_speed;
	double   _max_speed;

	Point2D  _position;
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
	Vector2D direction(void) const;

	uint32_t id(void) const;
	model_t model(void) const;
	
	void setEnvironment(std::shared_ptr<Environment> env);
	
	void update();

	void follow_path(std::list<Point2D>&);
	void random_walkway(std::list<Point2D>&);
	void follow_the_crowd(const Neighbors&);

	double distance(Agent const &_agent) const{
		return(sqrt(CGAL::squared_distance(this->_position,_agent._position)));
	}
};
#endif
