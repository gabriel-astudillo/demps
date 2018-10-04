#include <agent.hh>
#include <environment.hh>

std::shared_ptr<Environment> Agent::_myEnv;

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

void Agent::setEnvironment(std::shared_ptr<Environment> myEnv){
	this->_myEnv = myEnv;
}

void Agent::update(){

	switch(this->model()) {
		case SHORTESTPATH: {
		    this->shortestPath();
		    break;
		}
		case RANDOMWALKWAY: {
			this->randomWalkway();
		    break;
		}
		case FOLLOWTHECROWD: {
		    //Agent::Neighbors neighbors = _env.neighbors_of(this->_agents[i],g_attractionRadius,SHORTESTPATH);
			//
		    //if(neighbors.empty()){
		    //  if(this->_routes[this->_agents[i].id()].empty()){    
		    //     auto response = _router.route(this->_agents[i].position(),g_randomWalkwayRadius);
		    //     this->_routes[this->_agents[i].id()] = response.path();
		    //   }
		    //  this->_agents[i].random_walkway(this->_routes[this->_agents[i].id()]);
		    //}
		    //else
		    //   this->_agents[i].follow_the_crowd(neighbors);  
			break;
		}
		case WORKINGDAY: break;
		case SNITCH: break;
	}

}

void Agent::shortestPath(){
	//El agente ya tiene la ruta más corta asignada desde el inicio
	//de la simulación. Solo debe seguir dicha ruta.
	this->followPath();
}


void Agent::randomWalkway() {
	
	if(this->_route.empty()){  
		auto response = _myEnv->getRouter()->route(this->position(),g_randomWalkwayRadius);
		this->_route = response.path();
	}
    this->followPath();
}

void Agent::followPath(){
    static thread_local std::random_device device;
    static thread_local std::mt19937 rng(device());
	
    if(_route.empty()) return;

    std::uniform_real_distribution<double> speed(this->_min_speed,this->_max_speed);

    while(!_route.empty()) {
        Point2D dst = _route.front();
        double dist = sqrt(CGAL::squared_distance(this->_position,dst));

        if(dist < g_closeEnough) {
            _route.pop_front();
            continue;
        }

        Transformation scale(CGAL::SCALING,1.0,dist);
        Vector2D direction(this->_position,dst);
        direction = scale(direction);

        Transformation translate(CGAL::TRANSLATION,direction*speed(rng));
        this->_position = translate(this->_position);
        break;
    }

}

void Agent::followTheCrowd(const Neighbors &_neighbors){
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

Vector2D Agent::direction(void) const {
    return(this->_direction);
}

Point2D Agent::position(void) const {
    return(this->_position);
}

model_t Agent::model(void) const{
    return(this->_model);
}
