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

	this->_targetPos = Point2D(-666.6,-666.6);
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

/*
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
}*/

Agent::~Agent(void)
{
	;
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

const Vector2D Agent::currVelocity(void)
{
	return(this->_currVelocity);
}

void Agent::currVelocity(const Vector2D& velocity)
{
	this->_currVelocity = velocity;
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
		
		/*
		#pragma omp critical
		{
		std::cout << g_currTimeSim << ": " << this->id() <<  
			" => currQuad=" << currQuad   << std::endl;
		}*/
		
		
		_myEnv->getPatchAgent(currQuad)->delAgent( this->id() );

		this->setQuad(newQuad);
		
		/*
		#pragma omp critical
		{
		std::cout << g_currTimeSim << ": " <<  this->id() <<  ", x:" << this->_position[0] <<
			", y:" << this->_position[1] <<
			" => newQuad=" << newQuad   << std::endl;
		}
		*/
		
		

		_myEnv->getPatchAgent(newQuad)->addAgent( this->id() );
	}
}


Vector2D Agent::direction(void) const
{
	return(this->_direction);
}

void Agent::direction(const Vector2D& direction)
{
	this->_direction = direction;
}

uint32_t Agent::id(void) const
{
	return(this->_id);
}

model_t Agent::model(void) const
{
	return(this->_model);
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
		this->followTheCrowd();
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
		this->followPath();
		return;
	}
	
	//Lo del target 666.6 es algo cochino, pero la idea es
	//que si el agente copia la ruta de otro, yo no
	//entra en el ciclo de más abajo
	if(this->getTargetPos() != Point2D(-666.6,-666.6) ){
		this->followPath();
		return;
	}
	
	Agent::Neighbors agentNeighbors;
	_myEnv->setNeighborsOf(this->id(), g_attractionRadius, agentNeighbors);
	
	std::vector<Agent*> neighborsTofollow; // vecinos que se pueden seguir
	
	for(auto& fooAgent : agentNeighbors) {
		// Por el momento, los vecinos a seguir son del tipo "ShortestPath" o "FollowTheCrowd" RandomWalkway
		if( fooAgent->model() == ShortestPath || fooAgent->model() == FollowTheCrowd ){
			neighborsTofollow.push_back(fooAgent);
		}
	}
	
	// Si hay suficientes vecinos (p.e. 5), el agente pasa a ser "FollowTheCrowd" 
	if(neighborsTofollow.size() >= 5){
		//this->_model = FollowTheCrowd; //ShortestPath; //FollowTheCrowd;
		this->_route.clear();
		
		
		this->setTargetPos( neighborsTofollow[1]->getTargetPos() );
		this->currVelocity( neighborsTofollow[1]->currVelocity() );
		this->direction( neighborsTofollow[1]->direction() );
		
		//this->randomWalkwayForAdjustInitialPosition();
		auto response = _myEnv->getRouter()->route(this->position(), this->getTargetPos() );
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
		//double   deltaT       = 1.0;//[s]

		//Eq (2)
		//Helbing, D., & Molnar, P. (1998).
		//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
		DrivingForce = (_disiredSpeed * _direction - _currVelocity) / _timeRelax;
		
		// Por omisión, sólo actúa la fuerza DrivingForce
		_currVelocity += agentFactor * DrivingForce * g_deltaT;

		// Si a la actual velocidad, el agente va llegar al destino del tramo en menos
		// de g_deltaT tiempo, entonces se descarta el destino del tramo y se continua con el siguiente
		// destino de la ruta.
		if( dist / sqrt(_currVelocity.squared_length()) < g_deltaT ){
			_route.pop_front();
			continue;
		}
		
		
		/*if(dist < g_closeEnough) {
			_route.pop_front();
			continue;
		}*/

		

		Agent::Neighbors agentNeighbors;
		_myEnv->setNeighborsOf(this->id(), g_attractionRadius, agentNeighbors);
		//std::cout << g_currTimeSim << ": " <<  this->id() << ", Neighbors=>" << agentNeighbors.size() << std::endl;

		if( agentNeighbors.size() > 0 ){
			// Si hay vecinos, se debe considerar la SocialForce
			Vector2D totalRepulsiveEfect = Vector2D(0.0,0.0);

			// Determinar el efecto repulsivo por cada
			// vecino cercano, y agregarlo al total
			for(auto& fooAgent : agentNeighbors) {

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

				//if(CGAL::scalar_product(_direction, -repulsiveEfect) >= strengthRepulsiveEfect*_cosPhi) {
				//REVISAR ESTO ...OK
				if(CGAL::scalar_product(_direction, repulsiveEfect) >= strengthRepulsiveEfect*_cosPhi) {
					directionDependentWeight = 1;
				} else {
					directionDependentWeight = 0.5;
				}
				totalRepulsiveEfect += repulsiveEfect * directionDependentWeight;
				//totalRepulsiveEfect += repulsiveEfect;
			}
			
			//std::cout << g_currTimeSim << ": " <<  this->id() << ", totalRepulsiveEfect=>" << totalRepulsiveEfect << std::endl;

			//_currVelocity += agentFactor * (DrivingForce + totalRepulsiveEfect) * g_deltaT;
			_currVelocity += agentFactor * (totalRepulsiveEfect) * g_deltaT;
		}

		//Se limita la velocidad según Eq (11) y (12)
		//Helbing, D., & Molnar, P. (1998).
		//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
		// REVISAR Y COMPARAR CON
		// Chen, X., Treiber, M., Kanagaraj, V., & Li, H. (2018). Social force models for pedestrian traffic–state of the art. 
		// Transport Reviews.
		//if( sqrt(CGAL::scalar_product(_currVelocity, _currVelocity)) >= _maxDisiredSpeed ) {
		//	_currVelocity = _maxDisiredSpeed * _currVelocity / sqrt(CGAL::scalar_product(_currVelocity, _currVelocity));
		//}
		if( sqrt( _currVelocity.squared_length() ) >= _maxDisiredSpeed ) {
			_currVelocity = _maxDisiredSpeed * _currVelocity / sqrt( _currVelocity.squared_length() );
		}

		//Finalmente, se actualiza la posición del agente
		_position += _currVelocity * g_deltaT;
		
		//Actualizar el vector _direction
		dist = sqrt(CGAL::squared_distance(_position, dst));

		Transformation scaleNew(CGAL::SCALING, 1.0, dist);
		Vector2D directionNew(_position, dst);

		_direction = scaleNew(directionNew);
		
		// Si el agente se sale del área de simulación, dejarlo en el borde con velocidad 0.
		Environment::grid_t gridData = _myEnv->getGrid();
		if(_position[1] >= gridData._yMax || _position[1] <= gridData._yMin || _position[0] >= gridData._xMax || _position[0] <= gridData._xMin){
			_currVelocity = Vector2D(0.0,0.0);
			//_direction    = Vector2D(0.0,0.0);
			
			if(_position[1] >= gridData._yMax ){
				_position = Point2D(_position[0], gridData._yMax);
			}
			if(_position[1] <= gridData._yMin ){
				_position = Point2D(_position[0], gridData._yMin);
			}
			if(_position[0] >= gridData._xMax ){
				_position = Point2D(gridData._xMax, _position[1]);
			}
			if(_position[0] <= gridData._xMin){
				_position = Point2D(gridData._xMin, _position[1]);
			}
		}
		
		


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

void Agent::followTheCrowd()
{	
	this->followPath();
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
		double deltaT = 1.0;//[s]

		if(dist < g_closeEnough) {
			_route.pop_front();
			continue;
		}

		this->_currVelocity = _disiredSpeed * this->_direction * deltaT;

		//Transformation translate(CGAL::TRANSLATION, this->_currVelocity);
		//this->_position = translate(this->_position);
		
		_position += _currVelocity * deltaT;
		
		// Si el agente se sale del área de simulación, dejarlo en el borde con velocidad 0.
		Environment::grid_t gridData = _myEnv->getGrid();
		/*if(_position[1] >= gridData._yMax || _position[1] <= gridData._yMin || _position[0] >= gridData._xMax || _position[0] <= gridData._xMin){
			_currVelocity = Vector2D(0.0,0.0);
		}*/
		
		if(_position[1] >= gridData._yMax ){
			_position = Point2D(_position[0], gridData._yMax);
		}
		if(_position[1] <= gridData._yMin ){
			_position = Point2D(_position[0], gridData._yMin);
		}
		if(_position[0] >= gridData._xMax ){
			_position = Point2D(gridData._xMax, _position[1]);
		}
		if(_position[0] <= gridData._xMin){
			_position = Point2D(gridData._xMin, _position[1]);
		}

		break;
	}
}

double Agent::distanceTo(Agent* _agent) const
{
	return(sqrt(CGAL::squared_distance(this->_position,_agent->_position)));
}



