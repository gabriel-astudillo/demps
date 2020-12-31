#include <agent.hh>
#include <environment.hh>

std::shared_ptr<Environment> Agent::_myEnv;

Agent::Agent(void)
{
	this->_quad = 0;
}

Agent::Agent(const uint32_t &id, \
	const Point2D &position,\
	const std::string &initialZoneNameID, \
	const model_t &model,\
	const json& ageRange,\
	const json& phoneUse,\
	const json& SocialForceModel,\
	const json& responseTime)
{
	_id = id;
	
	_initialZoneNameID = initialZoneNameID;

	//Establecer el rango etareo del agente
	//
	//  Los grupos se establecen según el agrupamiento que realiza el INE para los censos.
	//  Por ejemplo, para Iquique:
	//    https://reportescomunales.bcn.cl/2017/index.php/Iquique
	//
	//  Las velocidades de cada grupo se establecen según la literatura:
	//  Gates et al. (2006)
	//  Makinoshima et al. (2018)
	
	_ageRange.groupFeatures[0].prob     = ageRange["G0"]["prob"].get<double>();
	_ageRange.groupFeatures[0].minSpeed = ageRange["G0"]["minSpeed"].get<double>();
	_ageRange.groupFeatures[0].maxSpeed = ageRange["G0"]["maxSpeed"].get<double>();
	_ageRange.groupFeatures[1].prob     = ageRange["G1"]["prob"].get<double>();
	_ageRange.groupFeatures[1].minSpeed = ageRange["G1"]["minSpeed"].get<double>();
	_ageRange.groupFeatures[1].maxSpeed = ageRange["G1"]["maxSpeed"].get<double>();
	_ageRange.groupFeatures[2].prob     = ageRange["G2"]["prob"].get<double>();
	_ageRange.groupFeatures[2].minSpeed = ageRange["G2"]["minSpeed"].get<double>();
	_ageRange.groupFeatures[2].maxSpeed = ageRange["G2"]["maxSpeed"].get<double>();
	_ageRange.groupFeatures[3].prob     = ageRange["G3"]["prob"].get<double>();
	_ageRange.groupFeatures[3].minSpeed = ageRange["G3"]["minSpeed"].get<double>();
	_ageRange.groupFeatures[3].maxSpeed = ageRange["G3"]["maxSpeed"].get<double>();
	
	//Determina grupo etario y su correspondiente vel min y max
	_ageRange.getAgeFeatures(_groupAge, _initSpeedRange.min, _initSpeedRange.max);

	_model     = model;
	_position  = position;
	
	_safeZoneNameID = "NA"; // Por omisión, la zona de seguridad no está asignada, se asigna después.
	_safeZone = nullptr;
	
	_targetPos = Point2D(-666.6,-666.6);
	_distanceToTargetPos = -1.0;
	
	_direction = Vector2D(0.0,0.0);
	_currVelocity = Vector2D(0.0,0.0);
	_radius    = 0.25;
	_density   = 0.0;
	
	// Si pertenece al grupo niño, se asume que va 
	// junto a un adulto ==> el radio se expande)
	if(_groupAge == 0) {
		_radius = 0.45;
	}
	
	_inSafeZone     = false;
	_evacuationTime = -1;
	_travelDistance = 0;

	
	_quad = this->determineQuad(); 
	
	//Establecer el tiempo de respuesta del agente
	// Para tiempo de respuesta, se utiliza una distribución Rayleigh.
	//   sigma: factor de escala. Representa cuán rápido los agentes salen de la fase de respuesta.
	//   tau  : tiempo de retardo. Antes de este tiempo, los agentes no inician el proceso de evacación.
	// Referencias:
	//     Mostafizi, A., Wang, H., Cox, D., Cramer, L. A., & Dong, S. (2017). 
	//       Agent-based tsunami evacuation modeling of unplanned network disruptions for 
	//       evidence-driven resource allocation and retrofitting strategies. Natural Hazards. 
	//     Mas, E., Suppasri, A., Imamura, F., & Koshimura, S. (2012). 
	//       Agent-based simulation of the 2011 great east japan earthquake/tsunami evacuation: 
	//       An integrated model of tsunami inundation and evacuation. Journal of Natural Disaster
	
	_responseTimeEngine.sigma = responseTime["sigma"].get<double>();
	_responseTimeEngine.tau   = responseTime["tau"].get<double>();
	
	
	_responseTime = _responseTimeEngine.getRayleigh();
	
	//Parametros para la simulación de Makinoshima (2018)
	//_responseTime = _responseTimeEngine.getLogNormalNumber(6.383,0.1655);
	
	_expo.lambda = 1.0/phoneUse["meanTimeTakePhone"].get<double>();
	
	_usePhone.probPhoneUseConst = phoneUse["probPhoneUseConst"].get<double>();;
	_usePhone.usingPhone = 0;
	_usePhone.probPhoneUse = 0.0;
	
	//Establecer la velocidad inicial del agente
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());
	std::uniform_real_distribution<double> speed(_initSpeedRange.min,_initSpeedRange.max);

	_SFM.disiredSpeed = speed(rng);
	_SFM.maxDisiredSpeed = 1.3 * this->_SFM.disiredSpeed;
	_SFM.timeRelax = SocialForceModel["timeRelax"].get<double>();//0.5; //[s]
	_SFM.sigma = SocialForceModel["sigma"].get<double>();//0.6;//[m]
	_SFM.strengthSocialRepulsiveForceAgents = SocialForceModel["repulsiveForceAgents"].get<double>(); //[m^2/s^2]
	_SFM.cosPhi = SocialForceModel["cosphi"].get<double>();//-0.93969 ; //cos(200º)

	
}

Agent::~Agent(void)
{
	;
}

void Agent::setTargetPos(const Point2D& tposition)
{
	_targetPos = tposition;
}

const Point2D Agent::getTargetPos(void) const
{
	return(this->_targetPos);
}

void Agent::setSafeZoneID(const std::string& safeZoneNameID)
{
	_safeZoneNameID = safeZoneNameID;
}

std::string Agent::getSafeZoneID()
{
	return(_safeZoneNameID);
}

std::string Agent::getInitialZoneID()
{
	return(_initialZoneNameID);
}	

void Agent::safeZone(Zone* safeZonePtr)
{
	_safeZone = safeZonePtr;
}

Zone* Agent::safeZone()
{
	return(_safeZone);
}

const double Agent::distanceToTargetPos()
{
	return(_distanceToTargetPos);
}

void Agent::distanceToTargetPos(const double& dist)
{
	_distanceToTargetPos = dist;
}

const Point2D Agent::position(void) const
{
	return(_position);
}

const Vector2D Agent::currVelocity(void)
{
	return(_currVelocity);
}

void Agent::currVelocity(const Vector2D& velocity)
{
	_currVelocity = velocity;
}

bool Agent::inSafeZone()
{
	return(_inSafeZone);
}

void Agent::inSafeZone(bool in)
{
	_inSafeZone = in;
}


void Agent::evacuationTime(uint32_t& currTick)
{
	if(currTick * g_deltaT <= _responseTimeEngine.tau){ //El agente ya está en zona segura antes de tau
		_evacuationTime = 0;
	}
	else{
		_evacuationTime = currTick * g_deltaT - _responseTimeEngine.tau;
	}
	
}

double Agent::evacuationTime()
{
	return(_evacuationTime);
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
	return(_direction);
}

void Agent::direction(const Vector2D& direction)
{
	_direction = direction;
}

uint32_t Agent::id(void) const
{
	return(_id);
}

double Agent::radius(void) const
{
	return(_radius);
}

model_t Agent::model(void) const
{
	return(_model);
}

uint32_t Agent::groupAge(void) const
{
	return(_groupAge);
}

double Agent::getDensity(void) const
{
	return(_density);
}

void Agent::setDensity(const double& density)
{
	_density = density;
}

double Agent::responseTime(void) const
{
	return(_responseTime);
}

void Agent::update()
{
	// El agente debe avanzar según su modelo de movilidad si ya
	// ha pasado su tiempo de respuesta
	
	if( g_currTimeSim >= (_responseTimeEngine.tau + _responseTime) ){//  && !this->inSafeZone()
		switch(this->model()) {
			case Residents: {
				this->shortestPath();
				break;
			}
			case Visitors_II: {
				this->randomWalkway();
				break;
			}
			case Visitors_I: {
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
		}
	}
	
	//Determina cuándo es el instante de tiempo
	//que debe "pensar" en utilizar su telefóno
	if( g_currTimeSim >= _usePhone.nextTimeUsePhone ){
		//Saca el telefono
			
		//La probabilidad de usar el telefono depende inversamente
		//de la cantidad de vecinos
		_usePhone.probPhoneUse = exp(-(double)_agentNeighbors.size() / _usePhone.probPhoneUseConst);
		
		std::random_device _randomDevice;
		std::uniform_real_distribution<> unifDistro(0.0, 1.0);
		double unifNumber = unifDistro(_randomDevice); 

		
		//Ve si realmente lo va a utilizar
		if(unifNumber <= _usePhone.probPhoneUse && !_route.empty() ) {
			_usePhone.usingPhone = 1;
		}
		else{
			_usePhone.usingPhone = 0;
		}
		
		//Actualiza el instante de tiempo cuando debe sacar el telefono
		this->setNextTimeUsePhone();
		
	}
	else{
		_usePhone.usingPhone = 0;
	}
	
	//Una vez que avanza, se calcula la distancia que le falta para llegar a su targetPos
	if( _safeZone != nullptr && !_route.empty() ){ //_safeZone
		_distanceToTargetPos = sqrt(CGAL::squared_distance(_position, this->getTargetPos() ));	
	}
	

}

void Agent::shortestPath()
{
	//El agente ya tiene la ruta más corta asignada desde el inicio
	//de la simulación. Solo debe seguir dicha ruta.
	this->followPath();

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
	
	_myEnv->setNeighborsOf(this->id(), g_attractionRadius); 
	
	std::vector<Agent*> neighborsTofollow; // vecinos que se pueden seguir
	
	for(auto& fooAgent : _agentNeighbors) {
		// Por el momento, los vecinos a seguir son del tipo "Residents" o "Visitors_I"
		if( fooAgent->model() == Residents || fooAgent->model() == Visitors_I ){
			neighborsTofollow.push_back(fooAgent);
		}
	}
	
	// Si hay suficientes vecinos (p.e. 5), el agente pasa a ser "Visitors type I" 
	if(neighborsTofollow.size() >= 5){
		_model = Visitors_I; 
		_route.clear();

		this->setTargetPos( neighborsTofollow[1]->getTargetPos() );
		this->currVelocity( neighborsTofollow[1]->currVelocity() );
		this->direction( neighborsTofollow[1]->direction() );
		this->setSafeZoneID( neighborsTofollow[1]->getSafeZoneID() );
		this->safeZone( neighborsTofollow[1]->safeZone() );
		
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

		Vector2D drivingForce = Vector2D(0.0,0.0);
		Vector2D totalRepulsiveEfect = Vector2D(0.0,0.0);
		Vector2D totalForce = Vector2D(0.0,0.0);
		
		//Eq (2) Helbing, D., & Molnar, P. (1998).
		//	Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
		drivingForce = (_SFM.disiredSpeed * _direction - _currVelocity) / _SFM.timeRelax;
		

		// Si a la actual velocidad, el agente va llegar al destino del tramo en menos
		// de g_deltaT tiempo, entonces se descarta el destino del tramo y se continua con el siguiente
		// destino de la ruta.
		if( dist / sqrt(_currVelocity.squared_length()) < g_deltaT ){
			_route.pop_front();
			continue;
		}
		
		_myEnv->setNeighborsOf(this->id(), g_attractionRadius); //Ya se actualizó en this->update();
		
		//Actualizar densidad de vecinos del agente
		this->setDensity(
			 (double)_agentNeighbors.size() / ( 3.14*(g_attractionRadius-this->radius())*(g_attractionRadius-this->radius()) )
				 );
		
		//std::cout << g_currTimeSim << ": " <<  this->id() << ", Neighbors=>" << _agentNeighbors.size() << std::endl;

		/**/
		if( _agentNeighbors.size() > 0 ){
			// Si hay vecinos, se debe considerar la RepulsiveForce
			
			// Determinar el efecto repulsivo por cada
			// vecino cercano, y agregarlo al total
			for(auto& fooAgent : _agentNeighbors) {

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

				double strengthRepulsiveEfect = _SFM.strengthSocialRepulsiveForceAgents * exp(-distance/_SFM.sigma);

				repulsiveEfect = -strengthRepulsiveEfect * directionAgentsUnit;
				
				
				// Determinar directionDependentWeight
				uint8_t directionDependentWeight = 1;

				//if(CGAL::scalar_product(_direction, -repulsiveEfect) >= strengthRepulsiveEfect*_cosPhi) {
				//REVISAR ESTO ...OK
				if(CGAL::scalar_product(_direction, repulsiveEfect) >= strengthRepulsiveEfect * _SFM.cosPhi) {
					directionDependentWeight = 1;
				} else {
					directionDependentWeight = 0.5; 
				}
				totalRepulsiveEfect += repulsiveEfect * directionDependentWeight;
				//totalRepulsiveEfect += repulsiveEfect;
			}
			
			//std::cout << g_currTimeSim << ": " <<  this->id() << ", totalRepulsiveEfect=>" << totalRepulsiveEfect << std::endl;

		}
		
		/**/
		totalForce =  drivingForce + totalRepulsiveEfect;
		_currVelocity +=  agentFactor * totalForce * g_deltaT;

		//Se limita la velocidad según Eq (11) y (12)
		//Helbing, D., & Molnar, P. (1998).
		//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
		// REVISAR Y COMPARAR CON
		// Chen, X., Treiber, M., Kanagaraj, V., & Li, H. (2018). Social force models for pedestrian traffic–state of the art. Transport Reviews.

		if( sqrt( _currVelocity.squared_length() ) >= _SFM.maxDisiredSpeed ) {
			_currVelocity = _SFM.maxDisiredSpeed * _currVelocity / sqrt( _currVelocity.squared_length() );
		}
		
		//Disminuir _currVelocity según la densidad de personas alrededor del agente
		
		if(this->getDensity() >=2.0){
			double velFactor = exp( -this->getDensity() )/exp(-2.0);
			if(velFactor <= 0.1){ velFactor = 0.1;}
			
			_currVelocity *= velFactor;
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
	/*static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());

	Vector2D direction(0.0,0.0);

	std::uniform_real_distribution<double> speed(this->_initSpeedRange.min,this->_initSpeedRange.max);

	for(auto& neighbor : _neighbors) {
		direction+=neighbor->direction();
	}

	Transformation scale(CGAL::SCALING,1.0,sqrt(direction.squared_length()));
	this->_direction=scale(direction);

	Transformation translate(CGAL::TRANSLATION,this->_direction*speed(rng));
	this->_position=translate(this->_position);*/
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
		double dist = sqrt(CGAL::squared_distance(_position, dst));

		Transformation scale(CGAL::SCALING, 1.0, dist);
		Vector2D direction(_position, dst);

		_direction = scale(direction);


		_currVelocity = _SFM.disiredSpeed * _direction;// * g_deltaT;
		
		
		// Si a la actual velocidad, el agente va llegar al destino del tramo en menos
		// de g_deltaT tiempo, entonces se descarta el destino del tramo y se continua con el siguiente
		// destino de la ruta.
		if( dist / sqrt(_currVelocity.squared_length()) < g_deltaT ){
			_route.pop_front();
			continue;
		}
		

		//Transformation translate(CGAL::TRANSLATION, this->_currVelocity);
		//this->_position = translate(this->_position);
		
		_position += _currVelocity * g_deltaT;
		
		// Si el agente se sale del área de simulación, dejarlo en el borde con velocidad 0.
		Environment::grid_t gridData = _myEnv->getGrid();
		
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

double Agent::distanceTo(Agent* fooAg) const
{
	return(abs(sqrt(CGAL::squared_distance(_position,fooAg->_position)) - _radius - fooAg->_radius));
}


void Agent::setNextTimeUsePhone()
{
	_usePhone.nextTimeUsePhone = _expo.exponentialTime();
}

double Agent::getNextTimeUsePhone()
{
	return(_usePhone.nextTimeUsePhone);
	
}

double Agent::getProbUsePhone()
{	
	return(_usePhone.probPhoneUse);
}

uint16_t Agent::getUsingPhone()
{
	return(_usePhone.usingPhone);
}

void Agent::setAgentNeighbors(const Neighbors& n){
	_agentNeighbors = n;
}

void Agent::clearAgentNeighbors(){
	_agentNeighbors.clear();
}

void Agent::addAgentNeighbors(Agent* ag){
	_agentNeighbors.push_back( ag );
}

Agent::Neighbors& Agent::getAgentNeighbors(){
	return(_agentNeighbors );
}

int Agent::getAgentNeighborsSize()
{
	return(_agentNeighbors.size() );
	
}


