#ifndef _AGENT_HH_
#define _AGENT_HH_
#include <glob.hh>

class Environment;

class Agent{
public:
	typedef std::vector<Agent*> Neighbors;
	static std::shared_ptr<Environment> _myEnv;
	
	std::list<Point2D> _route; // Primer intento. Debe ser un atributo privado.
	
private:
	uint32_t _id;
	double   _min_speed;
	double   _max_speed;
	
	model_t  _model;
	
	// Para los agentes que conocer donde su meta
	Point2D  _targetPos; 
	
	Neighbors _closeNeighbors;
	uint32_t _quad;
	
	//Variables para el modelo de Fuerza social
	//Helbing, D., & Molnar, P. (1998). 
	//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286. 
	
	Point2D  _position;
	Vector2D _direction;
	
	double   _disiredSpeed;
	double   _maxDisiredSpeed;
	Vector2D _currVelocity;
	
	//Driving force of the taget Point
	double   _timeRelax;
	
	//Interaction force between agents
	double   _sigma;    //[m]
	double   _strengthSocialRepulsiveForceAgents;
	double   _cosPhi; //cos(200º)


	
	

public:
	Agent(void);
	Agent(const Agent&);
	Agent(const uint32_t&,const Point2D&,const double&,const double&, const json& SocialForceModel, const model_t&);
 	Agent& operator=(const Agent&);
	
	~Agent(void);
	
	void setEnvironment(std::shared_ptr<Environment> env);

	void          setTargetPos(const Point2D& tposition);
	const Point2D getTargetPos(void) const;
	
	const Point2D  position(void) const;
	void     showPosition();
	uint32_t determineQuad();
	void     setQuad();
	void     setQuad(uint32_t idQuad);
	uint32_t getQuad() const;
	void     updateQuad();
	Vector2D direction(void) const;

	uint32_t id(void) const;
	model_t model(void) const;
	
	void clearCloseNeighbors();
	void addCloseNeighbors(Agent* neighbor);
	
	void update();

	void shortestPath();
	void followPath();
	void randomWalkway();
	void followTheCrowd(const Neighbors&);
	void randomWalkwayForAdjustInitialPosition();

	double distanceTo(Agent* _agent) const{
		return(sqrt(CGAL::squared_distance(this->_position,_agent->_position)));
	}
};
#endif
