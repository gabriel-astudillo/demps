#include <environment.hh>
#include <simulator.hh>


Environment::Environment(void)
{
	;
}

Environment::Environment(std::vector<Agent*> _vAgents)
{
	this->_vAgents = _vAgents;
}

Environment::Environment(const Environment &_env)
{
	//this->_tree=std::make_shared<kdtree>(*_env._tree);
}

Environment::~Environment(void)
{
	this->_initial_zones.clear();
	this->_reference_zones.clear();

}

/**
* @brief Crea el Router para los agentes en el mapa
*
*El Router se encarga de determinar las rutas que deben
*seguir los agentes en el mapa asignado.
*Necesita que el atributo _reference_point este inicializado.
*
* @param _map_osrm        : string que representa el archivo con la base de datos OSRM del mapa.
* @return void
*/
void Environment::setRouter(const std::string &_map_osrm)
{
	_router =  Router(_reference_point,_map_osrm);
}

Router* Environment::getRouter()
{
	return(&_router);
}

/**
* @brief Inicializa el punto de referencia del mapa
*
*El Punto de Referencia sirva para realizar la conversión
*entre coordenadas WGS84 y ENU. Marca el origen en este
*último sistema
*
* @param _fmap_zone: GeoJson de los datos del mapa
* @return void
*/

void Environment::setReferencePoint(const json &_fmap_zone)
{
	std::list<double> map_x;
	std::list<double> map_y;

	double xMin,yMin;

	for(auto& point : _fmap_zone["features"][0]["geometry"]["coordinates"][0]) {
		map_x.push_back( point[1] );
		map_y.push_back( point[0] );
	}
	map_x.sort();
	map_y.sort();

	//Coordenadas min  x e y.
	xMin = map_x.front();
	yMin = map_y.front();

	json geomData = {
		{"type", "Point"},
		{"coordinates", json::array({ yMin, xMin } ) }
	};

	this->_reference_point = {
		{"type", "FeatureCollection"},
		{"features", json::array({ json({{"type", "Feature"}, {"properties", {} }, {"geometry", geomData } }) }) }
	};

}
/*void Environment::setReferencePoint(const json &_freference_point){
	this->_reference_point = _freference_point;
}*/

/**
* @brief Crea el proyector para las coordenadas
*
*El proyector es una estructura que sirve para determinar
*donde se ubica el origen del sistema de coordenadas ENU.
*Necesita que el atributo _reference_point este inicializado.
*
* @param void
* @return void
*/
void Environment::setProjector()
{
	this->_projector = LocalCartesian(_reference_point["features"][0]["geometry"]["coordinates"][1],_reference_point["features"][0]["geometry"]["coordinates"][0],0,Geocentric::WGS84());
}

/**
* @brief Crea las zonas de referencia
*
*Las zonas de referencias son las zones seguras donde deben ir
*los agentes durante la simulación.
*
* @param _freference_zones: GeoJson que representa las zonas de referencia.
* @return void
*/
void Environment::setReferenceZones(const json &_freference_zones)
{
	for(auto& feature : _freference_zones["features"]) {
		this->_reference_zones.push_back(Zone(_reference_point, feature));
	}
}

/**
* @brief Crea las zonas iniciales
*
*Las zonas iniciales son las zones donde se van a
*ubicar los agentes al inicio de la simulación
*
* @param _finitial_zones: GeoJson que representa las zonas iniciales.
* @return void
*/
void Environment::setInitialZones(const json &_finitial_zones)
{
	for(auto& feature : _finitial_zones["features"]) {
		this->_initial_zones.push_back(Zone(_reference_point, feature));
	}
}

/**
* @brief Crea los cuadrantes del mapa de la simulación
*
* @param _fmap_zone: GeoJson que representa el mapa.
* @param quadSize  : tamaño de un lado del cuadrante, en metros.
* @return void
*/
void Environment::setGrid(const json &_fmap_zone, uint32_t quadSize)
{
	std::list<double> map_x;
	std::list<double> map_y;

	for(auto& point : _fmap_zone["features"][0]["geometry"]["coordinates"][0]) {
		double x,y,z;
		this->getProjector().Forward(point[1],point[0],0,x,y,z);

		map_x.push_back( x );
		map_y.push_back( y );
	}
	map_x.sort();
	map_y.sort();

	//Coordenadas min y max para x e y.
	_grid._xMin = map_x.front();
	_grid._xMax = map_x.back();
	_grid._yMin = map_y.front();
	_grid._yMax = map_y.back();

	//Ancho y alto del mapa
	_grid._mapWidth  = std::abs(_grid._xMax - _grid._xMin);
	_grid._mapHeight = std::abs(_grid._yMax - _grid._yMin);

	//Tamaño del cuadrante
	_grid._quadSize  = quadSize;

	//Cantidad de cuadrantes en el eje X e Y
	_grid._quadX = int(_grid._mapWidth / _grid._quadSize + 1);
	_grid._quadY = int(_grid._mapHeight / _grid._quadSize + 1);

}

Environment::grid_t Environment::getGrid()
{
	return(_grid);
}

void Environment::showGrid()
{
	std::cout <<
	          "quadSize:" << _grid._quadSize << "\n" <<
	          "xMin:" << _grid._xMin  << ", xMax:" << _grid._xMax << "\n" <<
	          "yMin:" << _grid._yMin  << ", yMax:" << _grid._yMax << "\n" <<
	          "mapWidth: " << _grid._mapWidth << ", mapHeight:" << _grid._mapHeight << "\n" <<
	          "quadX:" << _grid._quadX << ", quadY:"  << _grid._quadY << std::endl;
}

Zone Environment::getInitialZone(uint32_t id)
{
	return(this->_initial_zones[id]);
}

std::vector<Zone>  Environment::getInitialZones()
{
	return(this->_initial_zones);
}

std::vector<Zone>& Environment::getReferenceZones()
{
	return(this->_reference_zones);
}

LocalCartesian Environment::getProjector()
{
	return(this->_projector);
}

void Environment::addAgent(Agent* newAgent)
{
	this->_vAgents.push_back(newAgent);
}

void Environment::addPatchAgent(PatchAgent* newAgent) //NEW
{
	this->_pAgents.push_back(newAgent);
}

/**
* @brief Retorna el total de agentes del Environent
*
* @param void
* @return uint32_t
*/
uint32_t Environment::getTotalAgents()
{
	return(_vAgents.size());
}

/**
* @brief Retorna un puntero a un agente determinado
*
* @param uint32_t id: identificador del agente
* @return Agent*
*/
Agent* Environment::getAgent(uint32_t id)
{
	return(_vAgents[id]);
}

/**
* @brief Retorna un puntero a un patch agente determinado
*
* @param uint32_t id: identificador del patch agente
* @return PatchAgent*
*/
PatchAgent* Environment::getPatchAgent(uint32_t id)
{
	return(_pAgents[id]);
}


/**
* @brief Retorna un vector con los agentes el Environment
*
* @param void
* @return std::vector<Agent>
*/
std::vector<Agent*>& Environment::getAgents()
{
	return(_vAgents);
}

/**
* @brief Configura los vecinos de un agente identificado por su id.
* Al agente identificado se le asignan todos los agentes vecinos que
* están dentro del radio 'distanceMax'
*
* @param idAgent: entero que identifica el agente
* @param distanceMax: radio de la vecindad
* @param agentNeighbors: Vector con los agentes vecinos [out]
*/
void Environment::setNeighborsOf(const uint32_t& idAgent,const double& distanceMax, Agent::Neighbors& agentNeighbors)
{
	std::vector<uint32_t> idsAgents;

	Agent* agent = this->getAgent(idAgent);

	idsAgents = this->getPatchAgent( agent->getQuad() )->getAgents();

	//agent->clearCloseNeighbors();
	agentNeighbors.clear();

	for(auto& id : idsAgents) {
		if(id != idAgent) {
			Agent* neighbor;
			neighbor = this->getAgent(id);

			//agent->addCloseNeighbors(neighbor);

			if( neighbor == NULL ) {
				continue;
			}


			double dist = distance(agent, neighbor);
			if(  dist < distanceMax && dist > 0 ) {
				//agent->addCloseNeighbors(neighbor);
				agentNeighbors.push_back(neighbor);
			}


			//if( isClose(agent, neighbor, distanceMax) ){
			//	agent->addCloseNeighbors(neighbor);
			//}

		}
	}
}

/**
* @brief Ajusta la posición inicial de los agentes del Environment
*
*A cada agente que pertenezca al Environment, se le
*ajusta su posición incial en el mapa. Esto es debido a que
*la posición inicial del agente puede estar en un lugar donde
*no es necesariamente una calle. Esto se realiza en dos pasos:
*
* 1) Se ajusta el modelo de movilidad de todos los agentes a RANDOMWALK
* 2) Los agentes caminan un tiempo determinado para que finalmente
*    queden en las calles
*
*Este método es llamado por
*el método Simulator::calibrate()
*
* @param calibrationTime: tiempo de calibración
* @return void
*/
void Environment::adjustAgentsInitialPosition(const uint32_t& calibrationTime)
{
	//
	// AJUSTE: PASO 1
	//
	std::cout << "...(1/2)" << std::endl;

	ProgressBar pg;
	pg.start(this->getTotalAgents()-1);

	#pragma omp parallel for
	for(uint32_t i = 0; i < this->getTotalAgents(); i++) {
		if(g_showProgressBar) {
			pg.update(i);
		}

		Agent* agent = this->getAgent(i);

		auto response = this->getRouter()->route(agent->position(),g_randomWalkwayRadius);
		agent->_route = response.path();
	}

	if(g_showProgressBar) {
		std::cout << std::endl;
	}

	//
	// AJUSTE: PASO 2
	//
	std::cout << "...(2/2)" << std::endl;
	pg.start(calibrationTime-1);
	for(uint32_t t = 0; t < calibrationTime; t++) {
		if(g_showProgressBar) {
			pg.update(t);
		}

		#pragma omp parallel for
		for(uint32_t i = 0; i < this->getTotalAgents(); i++) {
			Agent* agent = this->getAgent(i);

			if(agent->_route.empty()) {
				auto response = this->getRouter()->route(agent->position(),g_randomWalkwayRadius);
				agent->_route = response.path();
			}
			agent->randomWalkwayForAdjustInitialPosition();
		}
	}
}

/**
* @brief Ajusta las reglas iniciales de los agentes del Environment
*
*A cada agente que pertenezca al Environment, se le
*ajusta sus reglas. Este método es llamado por
*el método Simulator::calibrate()
*
* @param void
* @return void
*/
void Environment::adjustAgentsRules()
{
	ProgressBar pg;
	pg.start(this->getTotalAgents()-1);

	#pragma omp parallel for
	for(uint32_t i = 0; i < this->getTotalAgents(); i++) {
		if(g_showProgressBar) {
			pg.update(i);
		}

		Agent* agent = this->getAgent(i);

		switch(agent->model()) {
		case ShortestPath: {
			double distance = DBL_MAX;
			Point2D  fooTarget;
			for(auto &reference_zone : this->getReferenceZones()) {
				/*
				//VERSION 1 ORIGINAL
				auto response = this->getRouter()->route(agent->position(),reference_zone.generate());
				if(response.distance() < distance) {
					distance = response.distance();
					agent->_route = response.path();
				}*/


				//VERSION 2
				// Por cada zona calcula en forma independiente la distancia agente-zona
				// Una vez que se obtiene la zona más cercana, se calcula la ruta hacia
				// ella.
				// Se logra un SpeedUp de 1.3 comparado con la V1
				/*
				fooTarget = reference_zone.generate();
				double fooDistance = this->getRouter()->distance(agent->position(), fooTarget);
				if( fooDistance < distance ){
					distance = fooDistance;
					agent->setTargetPos(fooTarget);
				}
				*/

				//VERSION 3
				// Similar a la C2, pero se basa en calcular la distancia
				// entre el agente y una zona de referencia
				// a través de la distancia euclideana. El error cometido es del orden
				// del 18% para el mapa de Iquique, Q1=12.04, Q3=21.62.
				// Se logra un SpeedUp de 2.8 comparado con la V1
				fooTarget = reference_zone.generate();
				double fooDistance = sqrt(CGAL::squared_distance(agent->position(), fooTarget));

				if( fooDistance < distance ) {
					distance = fooDistance;
					agent->setTargetPos(fooTarget);
				}

			}

			//VERSION 2, 3
			auto response = this->getRouter()->route(agent->position(),agent->getTargetPos());
			agent->_route = response.path();

			break;
		}
		case RandomWalkway:  {
			break;
		}
		case FollowTheCrowd:
			break;
		case WorkingDay:
			break;
		default: {
			std::cerr << "error::simulator_constructor::unknown_mobility_model::\"" << agent->model() << "\"" << std::endl;
			exit(EXIT_FAILURE);
		}
		}

	}
}

/**
* @brief Actualiza agentes del Environment
*
*Por cada agente que pertenezca al Environment, se
*actualiza su estado. Este método es llamado por
*Simulator::run()
*
* @param void
* @return void
*/
void Environment::updateAgents()
{

	uint32_t totalAgents = this->getTotalAgents();

	//Actualizar la posición de los agentes
	#pragma omp parallel for //schedule (dynamic,8)
	for(uint32_t i = 0; i < totalAgents; i++) {
		//std::cout << i << std::flush << std::endl;
		Agent* agent = this->getAgent(i);
		agent->update();
	}
}

/**
* @brief Actualiza el estado de la grilla
*
*Cada agente que pertenezca al Environment,
*actualiza los datos del cuadrante respectivo,
*avisando a los patchAgents correspondientes.
*Este método se llama después de updateAgents().
*
* @param void
* @return void
*/

void Environment::updateQuads()
{
	uint32_t totalAgents = this->getTotalAgents();

	#pragma omp parallel for //schedule (dynamic,8)
	for(uint32_t i = 0; i < totalAgents; i++) {
		Agent* agent = this->getAgent(i);
		agent->updateQuad();
	}
}

/**
* @brief Actualiza las estadisticas de la simulacion
*
*Este método se llama después de updateQuads().
*
* @param void
* @return void
*/
void Environment::updateStats()
{
	uint32_t totalAgents = this->getTotalAgents();

	#pragma omp parallel for //schedule (dynamic,8)
	for(uint32_t i = 0; i < totalAgents; i++) {
		Agent* agent = this->getAgent(i);

		for(auto& reference_zone : this->getReferenceZones() ) {
			bool isInside;
			isInside = reference_zone.pointIsInside(agent->position());

			//#pragma omp critical
			if(isInside) {
				reference_zone.addAgent(agent->id());
				reference_zone.updateAgentsDensity();
			} else {
				if(reference_zone.getAgentsDensity() > 0) {
					reference_zone.deleteAgent(agent->id());
					reference_zone.updateAgentsDensity();
				}
			}
		}
	}
}


double Environment::distance(Agent* a, Agent* b)
{
	return(sqrt(CGAL::squared_distance(a->position(),b->position())));
}

/*
bool Environment::isClose(Agent* a, Agent* b, const double& distanceMax)
{
	uint32_t dx, dy;

	dx = a->position()[0] - b->position()[0];
	dy = a->position()[1] - b->position()[1];


	return( (dx+dy) <=  (distanceMax + (dx*dy/distanceMax)) );
}
*/
