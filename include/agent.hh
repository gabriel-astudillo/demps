#ifndef _AGENT_HH_
#define _AGENT_HH_
#include <glob.hh>

class Environment;

class Agent
{
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
	
	Environment* _myEnv;

public:
    Agent(void);
    Agent(const Agent&);
    Agent(const uint32_t&,const Point2D&,const double&,const double&,const model_t&);
	Agent(const uint32_t&,const Point2D&,const double&,const double&,const model_t&, Environment* _myEnv);
    ~Agent(void);
    Agent& operator=(const Agent&);

    Point2D  position(void) const;
    Vector2D direction(void) const;

    void geographic(const GeoCoords&); //NO SE USA

    uint32_t id(void) const;
    model_t model(void) const;

    void follow_path(std::list<Point2D>&);
    void random_walkway(std::list<Point2D>&);
    void follow_the_crowd(const Neighbors&);

    inline value_type operator[](size_t const &k) const
    {
        return(this->_position[k]);
    }
    double distance(Agent const &_agent) const
    {
        return(sqrt(CGAL::squared_distance(this->_position,_agent._position)));
    }
};
#endif
