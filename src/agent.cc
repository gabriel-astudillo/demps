#include <agent.hh>
#include <environment.hh>

std::shared_ptr<Environment> Agent::_myEnv;

Agent::Agent(void)
{
	this->_quad = 0;
}

Agent::Agent(const uint32_t &_id, const Point2D &_position, const double &_min_speed, const double &_max_speed, const json& SocialForceModel, const model_t &_model)
{
	this->_id        = _id;
	this->_min_speed = _min_speed;
	this->_max_speed = _max_speed;
	this->_model     = _model;
	this->_position  = _position;
	this->_direction = Vector2D(0.0,0.0);

	this->_quad = this->determineQuad(); //AKA setQuad()


	//Establecer la velocidad del agente
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());

	std::uniform_real_distribution<double> speed(this->_min_speed,this->_max_speed);

	this->_disiredSpeed = speed(rng);
	this->_maxDisiredSpeed = 1.3 * this->_disiredSpeed;

	this->_currVelocity = Vector2D(0.0,0.0);

	this->_timeRelax =  SocialForceModel["timeRelax"].get<double>();//0.5; //[s]

	this->_sigma                              = SocialForceModel["sigma"].get<double>();//0.6;//[m]
	this->_strengthSocialRepulsiveForceAgents = SocialForceModel["repulsiveForceAgents"].get<double>();//2.1; //[m^2/s^-2]
	this->_cosPhi                             = SocialForceModel["cosphi"].get<double>();//-0.93969 ; //cos(200º)

}

Agent& Agent::operator=(const Agent &_agent)
{
	this->_id        = _agent._id;
	this->_min_speed = _agent._min_speed;
	this->_max_speed = _agent._max_speed;
	this->_model     = _agent._model;
	this->_position  = _agent._position;
	this->_direction = _agent._direction;
	this->_quad      = _agent._quad;
	this->_disiredSpeed = _agent._disiredSpeed;
	this->_maxDisiredSpeed = _agent._maxDisiredSpeed;
	this->_currVelocity    = _agent._currVelocity;
	this->_timeRelax       = _agent._timeRelax;
	this->_sigma           = _agent._sigma;
	this->_strengthSocialRepulsiveForceAgents = _agent._strengthSocialRepulsiveForceAgents;
	this->_cosPhi                             = _agent._cosPhi;
	return(*this);
}

Agent::~Agent(void)
{
	;
}

void Agent::setEnvironment(std::shared_ptr<Environment> myEnv)
{
	this->_myEnv = myEnv;
}

void Agent::setTargetPos(const Point2D& tposition)
{
	this->_targetPos = tposition;
}

const Point2D Agent::getTargetPos(void) const
{
	return(this->_targetPos);
}

const Point2D Agent::position(void) const
{
	return(this->_position);
}

void Agent::showPosition()
{
	std::cout << "t:" << g_currTimeSim << ", id:" << this->_id <<
	          ", x:" << this->_position[0] <<
	          ", y:" << this->_position[1] <<
	          ", Quad:" << this->getQuad() <<
	          ", AgentsInQuad:" <<  _myEnv->getPatchAgent(this->getQuad())->getAgents().size() << std::endl;
}

uint32_t Agent::determineQuad()
{
	uint32_t idQuad;
	Environment::grid_t gridData = _myEnv->getGrid();

	idQuad = (int)(this->_position[0] - gridData._xMin)/gridData._quadSize +
	         (int)((this->_position[1] - gridData._yMin) / gridData._quadSize) * gridData._quadX;

	return(idQuad);
}

void Agent::setQuad()
{
	_quad = this->determineQuad();
}

void Agent::setQuad(uint32_t idQuad)
{
	_quad = idQuad;
}

uint32_t Agent::getQuad() const
{
	return(this->_quad);
}

void Agent::updateQuad()
{
	uint32_t currQuad, newQuad;

	currQuad = this->getQuad();
	newQuad = this->determineQuad();

	if(currQuad != newQuad) { //Cambio de cuadrante
		_myEnv->getPatchAgent(currQuad)->delAgent( this->id() );

		this->setQuad(newQuad);

		_myEnv->getPatchAgent(newQuad)->addAgent( this->id() );
	}
}


Vector2D Agent::direction(void) const
{
	return(this->_direction);
}

uint32_t Agent::id(void) const
{
	return(this->_id);
}

model_t Agent::model(void) const
{
	return(this->_model);
}

void Agent::clearCloseNeighbors()
{
	_closeNeighbors.clear();
}

void Agent::addCloseNeighbors(Agent* neighbor)
{
	_closeNeighbors.push_back(neighbor);
}

void Agent::update()
{


	// El agente debe avanzar según su
	// modelo de movilidad
	switch(this->model()) {
	case ShortestPath: {
		this->shortestPath();
		break;
	}
	case RandomWalkway: {
		this->randomWalkway();
		break;
	}
	case FollowTheCrowd: {
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
	case WorkingDay:
		break;
	case SNITCH:
		break;
	}

}

void Agent::shortestPath()
{
	//El agente ya tiene la ruta más corta asignada desde el inicio
	//de la simulación. Solo debe seguir dicha ruta.
	this->followPath();

	/*
	if(_route.empty()) return;

	while(!_route.empty()) {
	    Point2D dst = _route.front();
	    double dist = sqrt(CGAL::squared_distance(this->_position, dst));

	    Transformation scale(CGAL::SCALING, 1.0, dist);
	    Vector2D direction(this->_position, dst);

		this->_direction = scale(direction);

	    if(dist < g_closeEnough) {
	        _route.pop_front();
	        continue;
	    }

		this->_currVelocity = _disiredSpeed * this->_direction;

		Transformation translate(CGAL::TRANSLATION, this->_currVelocity);
	    this->_position = translate(this->_position);

	    break;
	}
	*/

}

void Agent::randomWalkway()
{

	if(this->_route.empty()) {
		auto response = _myEnv->getRouter()->route(this->position(), g_randomWalkwayRadius);
		this->_route = response.path();
	}
	this->followPath();
}

void Agent::followPath()
{

	if(_route.empty()) {
		return;
	}

	double agentFactor = 1.0;

	while(!_route.empty()) {
		Point2D dst = _route.front();
		double dist = sqrt(CGAL::squared_distance(_position, dst));

		Transformation scale(CGAL::SCALING, 1.0, dist);
		Vector2D direction(_position, dst);

		_direction = scale(direction);

		Vector2D DrivingForce = Vector2D(0.0,0.0);
		double   deltaT       = 1.0;//[s]

		//Eq (2)
		//Helbing, D., & Molnar, P. (1998).
		//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
		DrivingForce = (_disiredSpeed * _direction - _currVelocity) / _timeRelax;

		if(dist < g_closeEnough) {
			_route.pop_front();
			continue;
		}

		_currVelocity += agentFactor * DrivingForce * deltaT;

		_myEnv->setNeighborsOf(this->id(), g_attractionRadius);

		//std::cout << g_currTimeSim << ": " <<  this->id() << "=>" << _closeNeighbors.size() << std::endl;

		if( _closeNeighbors.size() == 0 ) {
			// No hay agentes cercanos
			// Sólo actúa la fuerza DrivingForce

			_currVelocity += agentFactor * DrivingForce * deltaT;
		} else {

			Vector2D totalRepulsiveEfect = Vector2D(0.0,0.0);

			// Determinar el efecto repulsivo por cada
			// vecino cercano, y agregarlo al total
			for(auto& fooAgent : _closeNeighbors) {

				if( fooAgent == NULL || fooAgent == this ) {
					continue;
				}

				Vector2D repulsiveEfect = Vector2D(0.0,0.0);
				double   distance = distanceTo(fooAgent);

				Vector2D directionAgents(fooAgent->position(), _position);
				Vector2D directionAgentsUnit;


				directionAgentsUnit = directionAgents / sqrt(CGAL::scalar_product(directionAgents, directionAgents));

				//Helbing, D., & Molnar, P. (1998).
				//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
				// Determinar directionDependentWeight, Eq (8)

				double strengthRepulsiveEfect = _strengthSocialRepulsiveForceAgents * exp(-distance/_sigma);

				repulsiveEfect = -strengthRepulsiveEfect * directionAgentsUnit;

				// Determinar directionDependentWeight
				uint8_t directionDependentWeight = 1;

				if(CGAL::scalar_product(_direction, -repulsiveEfect) >= strengthRepulsiveEfect*_cosPhi) {
					directionDependentWeight = 1;
				} else {
					directionDependentWeight = 0.5;
				}
				totalRepulsiveEfect += repulsiveEfect * directionDependentWeight;
			}

			_currVelocity += agentFactor * (DrivingForce + totalRepulsiveEfect) * deltaT;
		}

		//Se limita la velocidad según Eq (11) y (12)
		//Helbing, D., & Molnar, P. (1998).
		//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
		// REVISAR Y COMPARAR CON
		//Chen, X., Treiber, M., Kanagaraj, V., & Li, H. (2018). Social force models for pedestrian traffic–state of the art. Transport Reviews.
		if( sqrt(CGAL::scalar_product(_currVelocity, _currVelocity)) >= _maxDisiredSpeed ) {
			_currVelocity = _maxDisiredSpeed * _currVelocity / sqrt(CGAL::scalar_product(_currVelocity, _currVelocity));
		}

		//Finalmente, se actualiza la posición del agente
		_position += _currVelocity * deltaT;

		break;
	}

}

void Agent::followTheCrowd(const Neighbors &_neighbors)
{
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());

	Vector2D direction(0.0,0.0);

	std::uniform_real_distribution<double> speed(this->_min_speed,this->_max_speed);

	for(auto& neighbor : _neighbors) {
		direction+=neighbor->direction();
	}

	Transformation scale(CGAL::SCALING,1.0,sqrt(direction.squared_length()));
	this->_direction=scale(direction);

	Transformation translate(CGAL::TRANSLATION,this->_direction*speed(rng));
	this->_position=translate(this->_position);
}

void Agent::randomWalkwayForAdjustInitialPosition()
{
	if(this->_route.empty()) {
		auto response = _myEnv->getRouter()->route(this->position(), g_randomWalkwayRadius);
		this->_route = response.path();
	}

	if(_route.empty()) {
		return;
	}

	while(!_route.empty()) {
		Point2D dst = _route.front();
		double dist = sqrt(CGAL::squared_distance(this->_position, dst));

		Transformation scale(CGAL::SCALING, 1.0, dist);
		Vector2D direction(this->_position, dst);

		this->_direction = scale(direction);

		if(dist < g_closeEnough) {
			_route.pop_front();
			continue;
		}

		this->_currVelocity = _disiredSpeed * this->_direction;

		Transformation translate(CGAL::TRANSLATION, this->_currVelocity);
		this->_position = translate(this->_position);

		break;
	}
}
