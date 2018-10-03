#include <environment.hh>

Environment::Environment(void) {
    ;
}

Environment::Environment(const std::vector<Agent> &_vAgents) {
	this->_vAgents = _vAgents;
}

Environment::Environment(const Environment &_env) {
	//this->_tree=std::make_shared<kdtree>(*_env._tree);
}

Environment::~Environment(void) {
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
void Environment::setRouter(const std::string &_map_osrm){
	_router =  Router(_reference_point,_map_osrm);
}

Router* Environment::getRouter(){
	return(&_router);
}

/**
* @brief Inicializa el punto de referencia del mapa
*
*El Punto de Referencia sirva para realizar la conversión
*entre coordenadas WGS84 y ENU. Marca el origen en este 
*último sistema
*
* @param _freference_point: GeoJson con los datos latitud y longitud del punto de referencia
* @return void
*/
void Environment::setReferencePoint(const json &_freference_point){
	this->_reference_point = _freference_point;
}

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
void Environment::setProjector(){
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
void Environment::setReferenceZones(const json &_freference_zones){
	for(auto& feature : _freference_zones["features"]){
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
void Environment::setInitialZones(const json &_finitial_zones){
	for(auto& feature : _finitial_zones["features"]){
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
void Environment::setGrid(const json &_fmap_zone, uint32_t quadSize){
	std::list<double> map_x;
	std::list<double> map_y;
	
	for(auto& point : _fmap_zone["features"][0]["geometry"]["coordinates"][0]){
		double x,y,z;
		this->getProjector().Forward(point[1],point[0],0,x,y,z);
	
		map_x.push_back( x );
		map_y.push_back( y );
	}
	map_x.sort(); map_y.sort();
	
	_xMin = map_x.front(); _xMax = map_x.back();	
	_yMin = map_y.front(); _yMax = map_y.back();
	
	//Ancho y alto del mapa
	_mapWidth  = std::abs(_xMax - _xMin);
	_mapHeight = std::abs(_yMax - _yMin);
	
	//Cantidad de cuadrantes en el eje X e Y
	_quadX = int(_mapWidth / _quadSize + 1);
	_quadY = int(_mapHeight / _quadSize + 1);

}

Zone Environment::getInitialZone(uint32_t id){
	return(this->_initial_zones[id]);
}

std::vector<Zone>  Environment::getInitialZones(){
	return(this->_initial_zones);
}

std::vector<Zone> Environment::getReferenceZones(){
	return(this->_reference_zones);
}

LocalCartesian Environment::getProjector(){
	return(this->_projector);
}

void Environment::addAgents(const std::vector<Agent> &_vAgents) {
	this->_vAgents = _vAgents;
}

/**
* @brief Retorna el total de agentes del Environent
*
* @param void
* @return uint32_t
*/
uint32_t Environment::getTotalAgents(){
	return(_vAgents.size());
}

/**
* @brief Retorna un puntero a un agente determinado
*
* @param uint32_t id: identificador del agente
* @return Agent*
*/
Agent* Environment::getAgent(uint32_t id){
	return(&_vAgents[id]);
}

/**
* @brief Retorna un vector con los agentes el Environment
*
* @param void
* @return std::vector<Agent>
*/
std::vector<Agent> Environment::getAgents(){
	return(_vAgents);
}

Agent::Neighbors Environment::neighbors_of(const Agent &_agent,const double &_max_distance,const model_t &_model) {
	Agent::Neighbors neighbors;
	/*
	std::deque<Agent> results;
	double dist=0.0;

	this->_tree->find_within_range(_agent,_max_distance,std::back_inserter(results));

	for (std::deque<Agent>::iterator it=results.begin(); it!=results.end(); ++it) {
	    Agent agent=*it;
	    if(agent.model()==_model){   
	        dist=_agent.distance(agent);
	        if(dist<=_max_distance)
	            neighbors.push_back(agent);
	    }
	}
	*/

	return(neighbors);
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
void Environment::adjustAgentsRules(){
	ProgressBar pg;
	pg.start(this->getTotalAgents()-1);
	
#pragma omp parallel for
	for(uint32_t i = 0; i < this->getTotalAgents(); i++){
		pg.update(i); //FALTA: condiciar visualización con la variable "showProgressBar"
			
		Agent* agent = this->getAgent(i);
		
		switch(agent->model()) {
			case SHORTESTPATH: {
				double distance = DBL_MAX;
				for(auto &reference_zone : this->getReferenceZones()) { 
					auto response = this->getRouter()->route(agent->position(),reference_zone.generate());
					if(response.distance() < distance) {
						distance = response.distance();
						this->_routes[agent->id()] = response.path();
					}
				}
				break;
			}
			case RANDOMWALKWAY:  {					            
				break;
			}
			case FOLLOWTHECROWD: break;
			case WORKINGDAY: break;
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
*actualza su estado. Este método es llamado por
*el método Simulator::run()
*
* @param void
* @return void
*/
void Environment::updateAgents(){
		
#pragma omp parallel for  //schedule(dynamic,8)  firstprivate(_router) shared(_env) 
	for(uint32_t i = 0; i < this->getTotalAgents(); i++){	
		Agent* agent = this->getAgent(i);
		
		agent->update();				
	}
		
}
