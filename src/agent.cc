#include <agent.hh>

Agent::Agent(void) {
    ;
}
Agent::Agent(const Agent &_agent) {
    this->_id=_agent._id;
    this->_min_speed=_agent._min_speed;
    this->_max_speed=_agent._max_speed;
    this->_model=_agent._model;
    this->_position=_agent._position;
    this->_direction=_agent._direction;
}
Agent::Agent(const uint32_t &_id,const Point2D &_position,const double &_min_speed,const double &_max_speed,const model_t &_model) {
    this->_id=_id;
    this->_min_speed=_min_speed;
    this->_max_speed=_max_speed;
    this->_model=_model;
    this->_position=_position;
    this->_direction=Vector2D(0.0,0.0);
}

Agent::Agent(const uint32_t &_id,const Point2D &_position,const double &_min_speed,const double &_max_speed,const model_t &_model, Environment* _myEnv) {
	this->_id        = _id;
	this->_min_speed = _min_speed;
	this->_max_speed = _max_speed;
	this->_model     = _model;
	this->_position  = _position;
	this->_direction = Vector2D(0.0,0.0);
	this->_myEnv     = _myEnv;
}

Agent& Agent::operator=(const Agent &_agent) {
    this->_id=_agent._id;
    this->_min_speed=_agent._min_speed;
    this->_max_speed=_agent._max_speed;
    this->_model=_agent._model;
    this->_position=_agent._position;
    this->_direction=_agent._direction;
    return(*this);
}
uint32_t Agent::id(void) const {
    return(this->_id);
}
Agent::~Agent(void) {
   ;
}
void Agent::follow_the_crowd(const Neighbors &_neighbors){
   static thread_local std::random_device device;
   static thread_local std::mt19937 rng(device());

   Vector2D direction(0.0,0.0);

   std::uniform_real_distribution<double> speed(this->_min_speed,this->_max_speed);

   for(auto& neighbor : _neighbors)
      direction+=neighbor.direction();

   Transformation scale(CGAL::SCALING,1.0,sqrt(direction.squared_length()));
   this->_direction=scale(direction);

   Transformation translate(CGAL::TRANSLATION,this->_direction*speed(rng));
   this->_position=translate(this->_position);
}
void Agent::random_walkway(std::list<Point2D> &_path) {
    this->follow_path(_path);
}
void Agent::follow_path(std::list<Point2D> &_path){
    static thread_local std::random_device device;
    static thread_local std::mt19937 rng(device());
    if(_path.empty()) return;

    std::uniform_real_distribution<double> speed(this->_min_speed,this->_max_speed);

    while(!_path.empty()) {
        Point2D dst=_path.front();
        double dist=sqrt(CGAL::squared_distance(this->_position,dst));

        if(dist<CLOSE_ENOUGH) {
            _path.pop_front();
            continue;
        }

        Transformation scale(CGAL::SCALING,1.0,dist);
        Vector2D direction(this->_position,dst);
        direction=scale(direction);

        Transformation translate(CGAL::TRANSLATION,direction*speed(rng));
        this->_position=translate(this->_position);
        break;
    }
}
Vector2D Agent::direction(void) const {
    return(this->_direction);
}
Point2D Agent::position(void) const {
    return(this->_position);
}
model_t Agent::model(void) const{
    return(this->_model);
}
