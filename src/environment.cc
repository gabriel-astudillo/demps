#include <environment.hh>

Environment::Environment(void) {
    //this->_tree=std::make_shared<kdtree>();
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

Environment::Environment(const std::vector<Agent> &_vAgents) {
    
	this->_vAgents = _vAgents;
	
	/*for(auto& agent : _agents) {
        _vAgents.push_back(agent);
	}*/
	
    /*
	this->_tree=std::make_shared<kdtree>();

    for(auto& agent : _agents) 
        this->_tree->insert(Agent(agent));

    this->_tree->optimise();
	*/
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

void Environment::update(const std::vector<Agent> &_agents) {
    /*
	this->_tree->clear();

    for(auto& agent : _agents) 
        this->_tree->insert(Agent(agent));

    this->_tree->optimise();
	*/
}
