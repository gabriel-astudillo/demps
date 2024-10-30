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
	const json& panicModel,\
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
	
	_safeZoneData.safeZoneNameID = "NA"; // Por omisión, la zona de seguridad no está asignada, se asigna después.
	_safeZoneData.safeZone = nullptr;
	_safeZoneData.targetPos = Point2D(-666.6,-666.6);
	_safeZoneData.distanceToTargetPos = -1.0;
	
	
	_direction = Vector2D(0.0,0.0);
	_currVelocity = Vector2D(0.0,0.0);
	_radius    = 0.25;
	_density   = 0.0;
	
	// Si pertenece al grupo niño, se asume que va 
	// junto a un adulto ==> el radio se expande)
	if(_groupAge == 0) {
		_radius = 0.45;
	}
	
	_evacuationData.inSafeZone     = false;
	_evacuationData.arrivedAtDestinationPoint = false;
	_evacuationData.evacuationTime = -1;
	_evacuationData.travelDistance = 0;
	_evacuationData.isWaiting = true;
	_evacuationData.isMoving = false;
	_evacuationData.isAlive = true;
	_evacuationData.isMovingRandomDueDebris = false;
	_evacuationData.isRouteRandom = false;
	_evacuationData.timeLived = -1;
	
	

	
	//_quad = this->determineQuad(); 
	_quad = _myEnv->getQuadId(this->position());
	_quadOld = _quad ;
	
	//_elevation = _myEnv->getPatchAgent(_quad)->getElevation();
	_gradient  = 0;
	
	//_newRouteByDebris = false;
	
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
	//_responseTime = _responseTimeEngine.getLogNormalNumber(6.383, 0.1655); // dt=1.00 <- Makinoshima
	//_responseTime = _responseTimeEngine.getLogNormalNumber(6.443, 0.1355); // dt=1.00 <- demps
	//_responseTime = _responseTimeEngine.getLogNormalNumber(6.403, 0.1055); // dt=0.01 <- demps
	
	_expo.lambda = 1.0/phoneUse["meanTimeTakePhone"].get<double>();
	
	_usePhone.probPhoneUseConst = phoneUse["probPhoneUseConst"].get<double>();;
	_usePhone.usingPhone = 0;
	_usePhone.probPhoneUse = 0.0;
	
	////////////////////////////////////////////
	//Establecer la velocidad inicial del agente
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());
	std::uniform_real_distribution<double> speed(_initSpeedRange.min,_initSpeedRange.max);

	////////////////////////////////////////////////
	// Cargar parámatros del modelo de fuerza social
	_SFM.disiredSpeed = speed(rng);
	_SFM.maxDisiredSpeed = 1.3 * this->_SFM.disiredSpeed;
	_SFM.timeRelax = SocialForceModel["timeRelax"].get<double>();//[s]
	_SFM.sigma = SocialForceModel["sigma"].get<double>();//[m]
	_SFM.strengthSocialRepulsiveForceAgents = SocialForceModel["repulsiveForceAgents"].get<double>(); //[m^2/s^2]
	_SFM.cosPhi = SocialForceModel["cosphi"].get<double>();//cos(200º)
	
	/////////////////////////////////////////
	// Cargar parámetros del modelo de pánico
	_panic.params.emotionThreshold           = panicModel["emotionThreshold"].get<double>();
	_panic.params.probInfectedToRecovered    = panicModel["probInfectedToRecovered"].get<double>();
	_panic.params.probRecoveredToSusceptible = panicModel["probRecoveredToSusceptible"].get<double>();
    _panic.params.meanTimeInInfected         = panicModel["meanTimeInInfected"].get<double>();
	_panic.params.sdTimeInInfected           = panicModel["sdTimeInInfected"].get<double>();
	_panic.params.meanTimeInRecovered        = panicModel["meanTimeInRecovered"].get<double>();
	_panic.params.sdTimeInRecovered          = panicModel["sdTimeInRecovered"].get<double>();
	
	std::random_device rd{};
	std::mt19937 gen{rd()};
	
	std::normal_distribution<> rTimeInInfected{_panic.params.meanTimeInInfected, _panic.params.sdTimeInInfected};
	std::normal_distribution<> rTimeInIRecovered{_panic.params.meanTimeInRecovered, _panic.params.sdTimeInRecovered};
	
	_panic.params.timeInInfected  = rTimeInInfected(gen);
	_panic.params.timeInRecovered = rTimeInIRecovered(gen);
		
}

Agent::~Agent(void)
{
	;
}

void Agent::setTargetPos(const Point2D& tposition)
{
	_safeZoneData.targetPos = tposition;
}

const Point2D Agent::getTargetPos(void) const
{
	return(this->_safeZoneData.targetPos);
}

void Agent::setSafeZoneID(const std::string& safeZoneNameID)
{
	_safeZoneData.safeZoneNameID = safeZoneNameID;
}

std::string Agent::getSafeZoneID()
{
	return(_safeZoneData.safeZoneNameID);
}

std::string Agent::getInitialZoneID()
{
	return(_initialZoneNameID);
}	

void Agent::safeZone(Zone* safeZonePtr)
{
	_safeZoneData.safeZone = safeZonePtr;
}

Zone* Agent::safeZone()
{
	return(_safeZoneData.safeZone);
}

const double Agent::distanceToTargetPos()
{
	return(_safeZoneData.distanceToTargetPos);
}

void Agent::distanceToTargetPos(const double& dist)
{
	_safeZoneData.distanceToTargetPos = dist;
}

void Agent::safeZoneDataIsFake(const bool& b)
{
	_safeZoneData.isFake = b;
}

bool Agent::safeZoneDataIsFake()
{
	return(_safeZoneData.isFake);
}

const Point2D Agent::position(void) const
{
	return(_position);
}

void Agent::position(const Point2D& position)
{
	_position = position;
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
	return(_evacuationData.inSafeZone);
}

void Agent::inSafeZone(bool in)
{
	_evacuationData.inSafeZone = in;
}

void Agent::isMoving(bool m)
{
	_evacuationData.isMoving = m;
}

bool Agent::isWaiting()
{
	return(_evacuationData.isWaiting);
}

void Agent::isWaiting(bool w)
{
	_evacuationData.isWaiting = w;
}

bool Agent::isMovingRandomDueDebris()
{
	return(_evacuationData.isMovingRandomDueDebris);
}

void Agent::isMovingRandomDueDebris(bool d)
{
	_evacuationData.isMovingRandomDueDebris = d;
}

bool Agent::isRouteRandom()
{
	return(_evacuationData.isRouteRandom);
}

void Agent::isRouteRandom(bool r)
{
	_evacuationData.isRouteRandom = r;
}

void Agent::isAlive(bool i)
{
	_evacuationData.isAlive = i;
}

bool Agent::isAlive()
{
	return(_evacuationData.isAlive);
}

void Agent::evacuationTime(uint32_t& currTick)
{
	if(currTick * global::params.deltaT <= _responseTimeEngine.tau){ //El agente ya está en zona segura antes de tau
		_evacuationData.evacuationTime = 0;
	}
	else{
		_evacuationData.evacuationTime = currTick * global::params.deltaT;// - _responseTimeEngine.tau;
	}
	
}

/*double Agent::evacuationTime()
{
	return(_evacuationData.evacuationTime);
}

double Agent::travelDistance()
{
	return(_evacuationData.travelDistance);
}*/

void Agent::showPosition()
{
	std::cout << "\x1B[0;90m";
	std::cout << "t:" << global::currTimeSim << ", id:" << this->_id <<
	          ", x:" << this->_position[0] <<
	          ", y:" << this->_position[1] <<
	          ", Quad:" << this->getQuad() <<
	          ", AgentsInQuad:" <<  _myEnv->getPatchAgent(this->getQuad())->getAgents().size() << std::endl;
	
	std::cout << "\x1B[0m" << std::endl;
}

void Agent::showPanic()
{
	std::cout << "\x1B[0;90m";
	std::cout << "t:" << global::currTimeSim << ", id:" << this->_id  << std::endl;
	std::cout << "Panic model enable : " << global::params.modelsEnable.panic << std::endl;
	std::cout << "Panic state" << std::endl;
	std::cout << "\t panic.state     : " << _panic.stateName() << std::endl;
	std::cout << "Panic params" << std::endl;
	std::cout << "\t panic.params.emotionThreshold           : " << _panic.params.emotionThreshold << std::endl;
	std::cout << "\t panic.params.probInfectedToRecovered    : " << _panic.params.probInfectedToRecovered << std::endl;
	std::cout << "\t panic.params.probRecoveredToSusceptible : " << _panic.params.probRecoveredToSusceptible << std::endl;
	std::cout << "\t panic.params.meanTimeInInfected          : " << _panic.params.meanTimeInInfected << std::endl;
	std::cout << "\t panic.params.meanTimeInRecovered         : " << _panic.params.meanTimeInRecovered << std::endl;
	std::cout << "Panic level (original)" << std::endl;
	std::cout << "\t panic.emotionOrig.strength  : " << _panic.emotionOrig.strength << std::endl;
	std::cout << "\t panic.emotionOrig.expression: " << _panic.emotionOrig.expression << std::endl;
	std::cout << "\t panic.emotionOrig.recv      : " << _panic.emotionOrig.recv << std::endl;
	std::cout << "\t panic.emotionOrig.send      : " << _panic.emotionOrig.send << std::endl;
	std::cout << "Panic level (current)" << std::endl;
	std::cout << "\t panic.emotion.strength  : " << _panic.emotion.strength << std::endl;
	std::cout << "\t panic.emotion.expression: " << _panic.emotion.expression << std::endl;
	std::cout << "\t panic.emotion.recv      : " << _panic.emotion.recv << std::endl;
	std::cout << "\t panic.emotion.send      : " << _panic.emotion.send << std::endl;
	std::cout << "\x1B[0m" << std::endl;
}


/*void Agent::setQuad()
{
	//_quad = this->determineQuad();
	_quad = _myEnv->getQuadId(this->position());
}*/

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
	newQuad = _myEnv->getQuadId(this->position());
	

	if(currQuad != newQuad) { //Cambio de cuadrante
		_myEnv->getPatchAgent(currQuad)->delAgent( this->id() );

		_quadOld = this->getQuad();
		this->setQuad(newQuad);

		_myEnv->getPatchAgent(newQuad)->addAgent( this->id() );
		
		if(global::params.modelsEnable.elevation){
			// Actualizar gradiente.
			this->updateGradient(currQuad, newQuad);
		}
	}
}

/*
	Actualizar la gradiente percibida por el agente.
	Este método es llamado por this->updateQuad()
*/
void Agent::updateGradient(uint32_t curPatch, uint32_t newPatch)
{
	int32_t curElevation = _myEnv->getPatchAgent(curPatch)->getElevation();
	int32_t newElevation = _myEnv->getPatchAgent(newPatch)->getElevation();
	
	if(newElevation >= 0){
		double m;
		
		m  = (double)(newElevation - curElevation);
		m /= (double)_myEnv->getGrid()._quadSize/2.0;
		this->setGradient(m);
		/*if(this->id() == 10000){
			std::cout << global::currTimeSim << "\tm:" << m;
			std::cout << ", curQuad: " << curQuad;
			std::cout << ", newQuad: " << newQuad;
			std::cout << ", currElevation:" << currElevation;
			std::cout << ", newElevation:" << newElevation;
			std::cout << std::endl; 
		}*/
		
		//this->setElevation(newElevation);
	}
	// Si es menor que 0, no se tiene info sobre la elevación
	// del patch, por lo que se mantiene la elevación y el gradiente del agente.
	
}

/*
void Agent::newRouteByDebris(bool d)
{
	_newRouteByDebris = d;
}
*/
/*
bool Agent::newRouteByDebris()
{
	return(_newRouteByDebris);
}
*/

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

/*
int32_t Agent::getElevation() const
{
	return(_elevation);
}
void Agent::setElevation(const int32_t elevation)
{
	_elevation = elevation;
}
*/

double Agent::getGradient() const
{
	return(_gradient);
}
void Agent::setGradient(const double gradient)
{
	_gradient = gradient;
}

double Agent::responseTime(void) const
{
	return(_responseTime);
}

void Agent::update()
{
	/*if(_id == 1000){
		std::cout << global::currTimeSim << ":\t";
		//std::cout << "emotion.strength: " << _panic.emotion.strength;
		std::cout << ", deceased: " << !_evacuationData.isAlive;
		std::cout << ", waiting: " << _evacuationData.isWaiting;
		std::cout << ", moving: " << _evacuationData.isMoving;
		std::cout << ", safed: " << _evacuationData.inSafeZone;
		//std::cout << ", panic state: " <<  _panic.stateName();
		std::cout << std::endl;
	}*/
	if(_myEnv->getFloodParams().enable && this->isAlive() && !this->inSafeZone() ){
		// Verificar si el patch está inundado.
		// Si su nivel de inundación es mayor o igual que el nivel crítico,
		// el agente no sigue evacuando y se considera fallecido.
		uint32_t patchId = this->getQuad();
		PatchAgent* pAgent = _myEnv->getPatchAgent(patchId);
		if( pAgent->getLevelFlood() >= _myEnv->getFloodParams().criticalLevel ){
			this->isAlive(false);
			this->isMoving(false);
			this->isWaiting(false);
			this->inSafeZone(false);
			_evacuationData.timeLived =  global::currTimeSim*global::params.deltaT;
		}
	}
	
	if(this->isAlive()){
		if( global::currTimeSim*global::params.deltaT >= (_responseTimeEngine.tau + _responseTime) ){
			if(this->inSafeZone()){
				this->isMoving(false);
				this->isWaiting(false);	
			}
			else{
				this->isMoving(true);
				this->isWaiting(false);	
			}
		}
		else{
			if(this->inSafeZone()){
				this->isMoving(false);
				this->isWaiting(false);	
			}
			else{
				this->isMoving(false);
				this->isWaiting(true);
			}		
		}
		
		if(!_evacuationData.isWaiting){
			
			if(global::params.modelsEnable.debris){
				//if(this->newRouteByDebris() && this->_route.empty() ){
				if(this->isMovingRandomDueDebris() && !this->isRouteRandom() ){
					//       |                                      |
					//       V                                      V
					//   es true cuando el agente            es true cuando el agente
					//   se encuentra con una celda          ya tiene una ruta aleatoria
					//   con escombros y no la               asignada debido a la condición
					//   puede atravesar.                    de la izquierda. Es para evitar
					//                                       que solicite más rutas aleatorias.
					//                                       Sólo puede pedir nuevamente cuando la
					//                                       ruta aleatoria asignada se acabe.
					
					// mover el agente al cuadrante anterior
					PatchAgent* pAgentOld = _myEnv->getPatchAgent(_quadOld);
					PatchAgent::quad_t quadOldInfo = pAgentOld->getQuadInfo();
					
					Point2D newPosition = Point2D(quadOldInfo.xc, quadOldInfo.yc);
					
					this->position(newPosition);
					
					this->isRouteRandom(true);
						
					auto response = _myEnv->getRouter()->route(this->position(), global::params.randomWalkwayRadius, true);
					//auto response = _myEnv->getRouter()->route(this->position(),this->getTargetPos());
					this->_route = response.path();	
					
					//std::cout << global::currTimeSim <<  "\t: " << this->id()  << "\t, ";
					//std::cout << "isMovingRandomDueDebris: " << this->isMovingRandomDueDebris() << ", ";
					//std::cout << "isRouteRandom: " << this->isRouteRandom() << " --> ";
					//std::cout << "new random route asigned due to debris";
					//std::cout << std::endl;								
				}
			}
			
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
					this->followTheCrowd();
					break;
				}
			}
		}
		
		//Determina cuándo es el instante de tiempo
		//que debe "pensar" en utilizar su telefóno
		if( global::currTimeSim*global::params.deltaT >= _usePhone.nextTimeUsePhone ){
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
	
		// Finalmente, se calcula la distancia que le falta para llegar a su targetPos, o
		// donde quedó de su targetPost si falleció :(
		if( _safeZoneData.safeZone != nullptr && !_route.empty() ){ //_safeZone
			_safeZoneData.distanceToTargetPos = sqrt(CGAL::squared_distance(_position, this->getTargetPos() ));	
		}
	
	}	
	
	/*
	// Finalmente, se calcula la distancia que le falta para llegar a su targetPos, o
	// donde quedó de su targetPost si falleció :(
	if( _safeZoneData.safeZone != nullptr && !_route.empty() ){ //_safeZone
		_safeZoneData.distanceToTargetPos = sqrt(CGAL::squared_distance(_position, this->getTargetPos() ));	
	}
	*/
	

}

void Agent::shortestPath()
{
	//El agente ya tiene la ruta más corta asignada desde el inicio
	//de la simulación. Solo debe seguir dicha ruta.
	this->followPath();

}

void Agent::randomWalkway()
{
	/*
	if( this->inSafeZone() ){
		return;
	}*/
	
	if(this->_route.empty()) {
		auto response = _myEnv->getRouter()->route(this->position(), global::params.randomWalkwayRadius);
		this->_route = response.path();
		this->followPath();
		return;
	}
	
	
	//if(this->safeZone() != nullptr && !this->safeZoneDataIsFake() ){
	if( !this->safeZoneDataIsFake()  ){
		this->followPath();
		return;
	}
	
	_myEnv->setNeighborsOf(this->id(), global::params.attractionRadius); 
	
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
		
		/////////////////////////////////////////////////////////////
		//Se elimina el agente actual de la zona asignada previamente
		this->safeZone()->deleteAgentAssigned(this->id());

		this->setTargetPos( neighborsTofollow[1]->getTargetPos() );
		this->currVelocity( neighborsTofollow[1]->currVelocity() );
		this->direction( neighborsTofollow[1]->direction() );
		this->setSafeZoneID( neighborsTofollow[1]->getSafeZoneID() );
		this->safeZone( neighborsTofollow[1]->safeZone() );
		this->safeZoneDataIsFake(false);
		
		auto response = _myEnv->getRouter()->route(this->position(), this->getTargetPos() );
		this->_route = response.path();	
		
		////////////////////////////////////////////////////////
		//Se agrega el agente actual a la nueva la zona asignada
		this->safeZone()->addAgentAssigned(this->id());
	}
	
	this->followPath();
	
}

void Agent::followPath()
{	
	if( this->inSafeZone() ){
		this->_panic.stateCode = Agent::panicState.susceptible;
		
		// Seguir sólo si se está visualizando la simulación
		if(global::execOptions.agentsOut == false){
			return;
		}
	}
	
	if(_myEnv->getFloodParams().enable){
		if(! this->isAlive()){
			return;
		}
	}

	if(_route.empty()) {
		if(global::params.modelsEnable.panic){
			if(this->_panic.stateCode == Agent::panicState.infected && _evacuationData.isMoving ){
				auto response = _myEnv->getRouter()->route(this->position(), global::params.randomWalkwayRadius);
				this->_route = response.path();
			}
		}
		else if( global::params.modelsEnable.debris && this->isMovingRandomDueDebris() ){
			this->isMovingRandomDueDebris(false);
			this->isRouteRandom(false);
			
			// Vuelve a la ruta
			auto response = _myEnv->getRouter()->route(this->position(),this->getTargetPos());
			this->_route = response.path();	
			
			//std::cout << global::currTimeSim <<  "\t: " << this->id()  << "\t, ";
			//std::cout << "isMovingRandomDueDebris: " << this->isMovingRandomDueDebris() << ", ";
			//std::cout << "isRouteRandom: " << this->isRouteRandom() << " --> ";
			//std::cout << "new fixed route asigned to safe zone";
			//std::cout << std::endl;			
		}
		else /*if(!_evacuationData.isWaiting)*/{
			return;
		}
	}
	

	Point2D dst = Point2D(0.0, 0.0);
	double agentFactor = 1.0;
	double dist = 0.0;
	
	Vector2D drivingForce = Vector2D(0.0,0.0);
	Vector2D totalRepulsiveEfect = Vector2D(0.0,0.0);
	Vector2D totalForce = Vector2D(0.0,0.0);
	
	/*if(std::isnan(_currVelocity[1] ) ){
		std::cout << global::currTimeSim << "\t" << this->id() << "\t, init) _currVelocity=" << _currVelocity << std::endl;
	}*/
	
	// Los vecinos son los que están a una distancia menor que 'global::params.attractionRadius'
	// y que estén vivos
	_myEnv->setNeighborsOf(this->id(), global::params.attractionRadius);
	
	if(!_route.empty() && !_evacuationData.isWaiting) { 
		dst = _route.front();

		dist = sqrt(CGAL::squared_distance(_position, dst));

		Transformation scale(CGAL::SCALING, 1.0, dist);
		Vector2D direction(_position, dst);

		_direction = scale(direction);
		
		// Si a la actual velocidad, el agente va llegar al destino del tramo en menos
		// de global::params.deltaT tiempo, entonces se descarta el destino del tramo y se continua con el siguiente
		// destino de la ruta.
		if( dist / sqrt(_currVelocity.squared_length()) < global::params.deltaT ){
			_route.pop_front();
			//continue;						
		}
			
		//Eq (2) Helbing, D., & Molnar, P. (1998).
		//	Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
		drivingForce = (_SFM.disiredSpeed * _direction - _currVelocity) / _SFM.timeRelax;
		
		
		//Actualizar densidad de vecinos del agente
		_myEnv->setDensityOf(this);
	}
		

	if(global::params.modelsEnable.panic){
		switch(this->_panic.stateCode){
			case Agent::panicState.susceptible:{
				this->_panic.emotion.expression = 0.6 * this->_panic.emotionOrig.expression;
				this->_panic.emotion.send       = 0.8 * this->_panic.emotionOrig.send;
				this->_panic.emotion.recv       = 0.8 * this->_panic.emotionOrig.recv;
				break;
			}
		
			case Agent::panicState.infected:{
				this->_panic.emotion.expression = this->_panic.emotionOrig.expression;
				this->_panic.emotion.send       = this->_panic.emotionOrig.send;
				this->_panic.emotion.recv       = this->_panic.emotionOrig.recv;		
				break;
			}
		
			case Agent::panicState.recovered:{
				this->_panic.emotion.expression = 0.8 * this->_panic.emotionOrig.expression;
				this->_panic.emotion.send       = 0.6 * this->_panic.emotionOrig.send;
				this->_panic.emotion.recv       = 0.8 * this->_panic.emotionOrig.recv;		
				break;
			}
		
		}
	}
		

	if( _agentNeighbors.size() > 0 ){
		// Si hay vecinos, se debe considerar:
		//   1) el contagio de pánico
		//   2) el cálculo de fuerzas del modelo SFM

		double newStrengthFactor = 0.0;
		for(auto& fooAgent : _agentNeighbors) {

			if( fooAgent == NULL || fooAgent == this || fooAgent->position() == this->position()) {
				continue;
			}
			
			//////////////////////////////////////////////
			// Modelo de contagio de pánico
			// parte 1: contagio
			if(global::params.modelsEnable.panic){
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// El nivel emocional depende del receptor depende de: a) la distancia entre ellos
				//                                                     b) el nivel de expresión del agente emisor
				//                                                     c) el nivel de receptividad del agente receptor
				//                                                     d) el nivel emocional del agente emisor
				double distance   = distanceTo(fooAgent);
				double distFactor =( 1 - 1/(1+exp(-distance))) * 0.15;
				double emotionFactor = fooAgent->_panic.emotion.expression * this->_panic.emotion.recv;// * fooAgent->_panic.emotion.send;
				
				double deltaStrength = distFactor*emotionFactor;

				// Si el modelo de inundación está habilitado, entonces
				// la variación del nivel emocional se incrementa en forma
				// lineal
				/*
				if(_myEnv->getFloodParams().enable){
					double floodFactor = 1.0;
					
					uint32_t patchId = this->getQuad();
					PatchAgent* pAgent = _myEnv->getPatchAgent(patchId);

					floodFactor = 1 + pAgent->getLevelFlood() / _myEnv->getFloodParams().criticalLevel;
					
					if(floodFactor > 2.0){
						floodFactor = 2.0;
					}
					
					deltaStrength *= floodFactor;
				}
				*/
				
				if(fooAgent->_panic.emotion.strength > 0){
				   newStrengthFactor += deltaStrength*fooAgent->_panic.emotion.strength;
				}else{
				   newStrengthFactor += deltaStrength;
				}
				
				
			}
			

			//////////////////////////////////////////////
			// Modelo de movilidad basado en fuerza social
			// parte 1: cálculo de fuerzas
			if(!this->isWaiting()){
				Vector2D repulsiveEfect = Vector2D(0.0,0.0);
				double   distance = distanceTo(fooAgent);

				Vector2D directionAgents(fooAgent->position(), _position);
				double directionAgentsMag = sqrt(directionAgents.squared_length());
				
				Vector2D directionAgentsUnit;

				if(directionAgentsMag > 0){
					directionAgentsUnit = directionAgents / directionAgentsMag;

					//Helbing, D., & Molnar, P. (1998).
					//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
					// Determinar directionDependentWeight, Eq (8)

					double strengthRepulsiveEfect = _SFM.strengthSocialRepulsiveForceAgents * exp(-distance/_SFM.sigma);

					repulsiveEfect = -strengthRepulsiveEfect * directionAgentsUnit;
				
					//////////////////////////////////////
					// Determinar directionDependentWeight
					uint8_t directionDependentWeight = 1;

					if(CGAL::scalar_product(_direction, repulsiveEfect) >= -strengthRepulsiveEfect * _SFM.cosPhi) {
						directionDependentWeight = 1;
					} else {
						directionDependentWeight = 0.5; 
					}
					totalRepulsiveEfect += repulsiveEfect * directionDependentWeight;	
				}
							
			}
			
		} // fin for(auto& fooAgent : _agentNeighbors)
		
		if(global::params.modelsEnable.panic){
			this->_panic.emotion.strength += newStrengthFactor / (double)_agentNeighbors.size();	
		}
	}
		
	///////////////////////////////////////////////////////
	// Modelo de contagio de pánico
	// parte 2: transiciones
	if(global::params.modelsEnable.panic){
		//Normalizar el nivel de emocion
		double strengthNorm = this->_panic.emotion.strength / (1 +  this->_panic.emotion.strength);
		
		if(this->_panic.stateCode == Agent::panicState.susceptible){
			if(!this->inSafeZone()){
				this->_panic.susceptibleTime += global::params.deltaT;

				if(strengthNorm >= this->_panic.params.emotionThreshold){
					this->_panic.stateCode        = Agent::panicState.infected;
					this->_panic.susceptibleTime  = 0.0;
					//this->_panic.emotion.strength = 0.0;
					this->_panic.infectedTime     = 0.0;
					this->_panic.recoveredTime    = 0.0;
		
					// Cuando se infecta, se le olvida la ruta y camina en forma aleatoria
					if(!_evacuationData.isWaiting){
						auto response = _myEnv->getRouter()->route(this->position(), global::params.randomWalkwayRadius);
						this->_route = response.path();
					}
				}
			}
		}
		else
		if(this->_panic.stateCode == Agent::panicState.infected){
			std::random_device device;
			std::uniform_real_distribution<> unifNumber(0.01, 1.0);
		
			// Condición para salir del estado INFECTED y pasar a RECOVERED
			if(this->_panic.infectedTime > this->_panic.params.timeInInfected &&
			    unifNumber(device) < this->_panic.params.probInfectedToRecovered){
	   			this->_panic.stateCode        = Agent::panicState.recovered;
				this->_panic.susceptibleTime  = 0.0;
	   			this->_panic.emotion.strength = 0.0;
	   			this->_panic.infectedTime     = 0.0;
	   			this->_panic.recoveredTime    = 0.0;	
			
				// Cuando se recupera, recalcula la ruta a su zona segura
				if(!_evacuationData.isWaiting){
					auto response = _myEnv->getRouter()->route(this->position(),this->getTargetPos());
					this->_route = response.path();		
				}	
			}
			else{
				this->_panic.infectedTime += global::params.deltaT;
			}
		}
		else
		if(this->_panic.stateCode == Agent::panicState.recovered){
			std::random_device device;
			std::uniform_real_distribution<> unifNumber(0.01, 1.0);
		
			if(this->_panic.recoveredTime > this->_panic.params.timeInRecovered &&
			    unifNumber(device) < this->_panic.params.probRecoveredToSusceptible){
	   			this->_panic.stateCode        = Agent::panicState.susceptible;
				this->_panic.susceptibleTime  = 0.0;
	   			this->_panic.emotion.strength = 0.0;
	   			this->_panic.infectedTime     = 0.0;
	   			this->_panic.recoveredTime    = 0.0;	
			
			}
			else{
				this->_panic.recoveredTime += global::params.deltaT;
			}
		}
	}
		
	if(!_route.empty() && !_evacuationData.isWaiting) {
		// Orden original
		//  _myEnv->setGradientVelocityOf(this)
		//  SFM
		//  _myEnv->setFloodVelocityOf(this);
		//  _myEnv->setDensityVelocityOf(this);
		//  _myEnv->setDebrisVelocityOf(this);
				

		//////////////////////////////////////////////////////////
		// Disminuir _currVelocity según la gradiente del terreno.
		if(global::params.modelsEnable.elevation){
			_myEnv->setGradientVelocityOf(this);
		}
		
		
		//////////////////////////////////////////////
		// Modelo de movilidad basado en fuerza social
		// parte 2: actualización de la posición
		totalForce =  drivingForce + totalRepulsiveEfect;
		_currVelocity +=  agentFactor * totalForce * global::params.deltaT;
		

		//////////////////////////////////////////////
		//Se limita la velocidad según Eq (11) y (12)
		//Helbing, D., & Molnar, P. (1998).
		//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
		if( sqrt( _currVelocity.squared_length() ) >= _SFM.maxDisiredSpeed ) {
			_currVelocity = _SFM.maxDisiredSpeed * _currVelocity / sqrt( _currVelocity.squared_length() );
		}
		
		/*
		//////////////////////////////////////////////////////////////////////////////
		//Disminuir _currVelocity según la densidad de personas alrededor del agente
		if(_myEnv->getDensityParams().enable){
			_myEnv->setDensityVelocityOf(this);
		}
		*/
		
		
		
		if(_myEnv->getFloodParams().enable){
			// Determinar el nivel de inundación del patch.
			// Según este nivel, la velocidad disminuye en un factor inversamente proporcional, hasta un factor
			// determinado de la velocidad inicial 'disiredSpeed' especificada en el modelo de Fuerza social (estructura this->_SFM)
			_myEnv->setFloodVelocityOf(this);		
		}
		
		
		
		
		//////////////////////////////////////////////////////////////////////////////
		// Disminuir _currVelocity según la cantidad de escombros del patch
		if(global::params.modelsEnable.debris ){
			uint32_t patchId = this->getQuad();
			PatchAgent* pAgent = _myEnv->getPatchAgent(patchId);
			
			
			if( !pAgent->isDebrisFree() && !this->isMovingRandomDueDebris() && !this->isRouteRandom() ){
				bool seekNewRoute;
				
				seekNewRoute = _myEnv->setDebrisVelocityOf(this);
				
				// Si el agente debe buscar nueva ruta debido a que se encontró 
				// con escombros, se activa el flag 'isMovingRandomDueDebris'. En el próximo tick
				// deberá determinar una nueva ruta.
				this->isMovingRandomDueDebris(seekNewRoute);
			}		
		}
		
		//////////////////////////////////////////////////////////////////////////////
		//Disminuir _currVelocity según la densidad de personas alrededor del agente
		if(_myEnv->getDensityParams().enable){
			_myEnv->setDensityVelocityOf(this);
		}
		
		
		
	
		//
		Point2D positionOld = Point2D(0.0, 0.0);
		positionOld = _position;
		
		//////////////////////////////////////////////////
		//Finalmente, se actualiza la posición del agente
		_position += _currVelocity * global::params.deltaT;
		
		//////////////////////////////////////////////////
		//Se actualiza la distancia que ha recorrido el agente
		if( !this->inSafeZone() ){
			_evacuationData.travelDistance += sqrt(CGAL::squared_distance(_position, positionOld));;
		}
		
		//////////////////////////////////////////////////
		//Actualizar el vector _direction
		dist = sqrt(CGAL::squared_distance(_position, dst));


		Transformation scaleNew(CGAL::SCALING, 1.0, dist);
		Vector2D directionNew(_position, dst);

		_direction = scaleNew(directionNew);
		
		////////////////////////////////////////////////////////////////////////////////////
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
		auto response = _myEnv->getRouter()->route(this->position(), global::params.randomWalkwayRadius);
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


		_currVelocity = _SFM.disiredSpeed * _direction;// * global::params.deltaT;
		
		
		// Si a la actual velocidad, el agente va llegar al destino del tramo en menos
		// de global::params.deltaT tiempo, entonces se descarta el destino del tramo y se continua con el siguiente
		// destino de la ruta.
		if( dist / sqrt(_currVelocity.squared_length()) < global::params.deltaT ){
			_route.pop_front();
			continue;
		}
		
		_position += _currVelocity * global::params.deltaT;
		
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

Agent::panic_t Agent::getPanicStruct()
{
	return(_panic);
}

Agent::evacuationData_t Agent::getEvacuationDataStruct()
{
	return(_evacuationData);
}

void Agent::setEvacuationDataStruct(Agent::evacuationData_t eData)
{
	_evacuationData = eData;
}


Agent::SFM_t Agent::getSFMstruct()
{
	return(_SFM);
}







