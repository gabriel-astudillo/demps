#include <environment.hh>

Environment::Environment(void) {
    ;
}

Environment::Environment(double xMin, double xMax, double yMin, double yMax, uint32_t quadSize){
	_xMin = xMin;
	_xMax = xMax;
	_yMin = yMin;
	_yMax = yMax;
	_quadSize = quadSize;
	
	_mapWidth  = std::abs(_xMax - _xMin);
	_mapHeight = std::abs(_yMax - _yMin);
	
	//Cantidad de cuadrantes en el eje X e Y
	_quadX = int(_mapWidth / _quadSize + 1);
	_quadY = int(_mapHeight / _quadSize + 1);
}

void Environment::setRouter(const json &_freference_point,const std::string &_map_osrm){
	_router =  Router(_freference_point,_map_osrm);
}

Router Environment::getRouter(){
	return(_router);
}

Environment::Environment(const std::vector<Agent> &_vAgents) {
	this->_vAgents = _vAgents;
}

Environment::Environment(const Environment &_env) {
	//this->_tree=std::make_shared<kdtree>(*_env._tree);
}
Environment::~Environment(void) {
	//this->_tree->clear();
}

Environment& Environment::operator=(const Environment &_env) {
	//this->_tree=std::make_shared<kdtree>(*_env._tree);
	return(*this);
}


void Environment::addAgents(const std::vector<Agent> &_vAgents) {
	this->_vAgents = _vAgents;
}

uint32_t Environment::getTotalAgents(){
	return(_vAgents.size());
}

Agent* Environment::getAgent(uint32_t id){
	return(&_vAgents[id]);
}

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

void Environment::updateAgents(){
		
#pragma omp parallel for  //schedule(dynamic,8)  firstprivate(_router) shared(_env) 
	for(uint32_t i = 0; i < this->getTotalAgents(); i++){	
		Agent* agent = this->getAgent(i);
		
		agent->update();				
	}
		
}
