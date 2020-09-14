#include <patchAgent.hh>
#include <environment.hh>

std::shared_ptr<Environment> PatchAgent::_myEnv;


PatchAgent::PatchAgent(const uint32_t &id)
{
	this->_id = id;

}


PatchAgent::~PatchAgent(void)
{
	;
}

uint32_t PatchAgent::getId(){
	return(_id);
}

void PatchAgent::addAgent(const uint32_t &idAgent)
{
	std::lock_guard<std::mutex> l(_mtx);
	_agentsInPatch.push_back(idAgent);

}

void PatchAgent::delAgent(const uint32_t &idAgent)
{

	//
	// Buscar identificador del agente en el patch y eliminarlo
	// de la lista _agentsInPatch
	//
	std::lock_guard<std::mutex> l(_mtx);
	for (auto it = _agentsInPatch.begin(); it != _agentsInPatch.end();) {
		if (*it == idAgent) {
			it = _agentsInPatch.erase(it);
			break;
		} else {
			++it;
		}
	}
}

std::vector<uint32_t>& PatchAgent::getAgents()
{	
	return(_agentsInPatch);
}


std::vector<uint32_t>& PatchAgent::getNeighborsAgents()
{	
	/*
	idPatchNeighbors patchNeighbors = this->findPatchNeighbors4();
	
	std::vector<uint32_t> NeighborsAgents;
	NeighborsAgents = this->getAgents();
	
	
	for(const auto& fooIdPatch : patchNeighbors){
		std::vector<uint32_t> newNeighborsAgents;
		newNeighborsAgents = _myEnv->getPatchAgent( fooIdPatch )->getAgents();
		NeighborsAgents.insert(NeighborsAgents.end(), newNeighborsAgents.begin(), newNeighborsAgents.end());

	}*/
	
	/*std::lock_guard<std::mutex> l(_mtx);
	_neighborsAgents.clear();
	_neighborsAgents = NeighborsAgents;
	return(_neighborsAgents);*/
	
	
	return(_agentsInPatch);

	
	
}

PatchAgent::idPatchNeighbors PatchAgent::findPatchNeighbors4()
{
	idPatchNeighbors fooNeigh;
	
	int32_t idNeighbors[4];
	
	Environment::grid_t gridData = _myEnv->getGrid();
	
	idNeighbors[0] = getId() + gridData._quadX; //N
	idNeighbors[1] = getId() - gridData._quadX; //S
	idNeighbors[2] = getId() - 1; //W
	idNeighbors[3] = getId() + 1; //E
	
	uint32_t Xoffset = getId() % gridData._quadX;
	
	if(getId() >= gridData._quadX * (gridData._quadY - 1)){ //Sin vecinos N
		idNeighbors[0] = -1;
	}
	
	if(getId() >= 0 && getId() < gridData._quadX ){ // Sin vecinos S
		idNeighbors[1] = -1;
	}
	
	if(Xoffset == 0){ // Sin vecinos W
		idNeighbors[2] = -1;
	}
	
	if(Xoffset == (gridData._quadX - 1)){ // Sin vecinos E
		idNeighbors[3] = -1;
	}
	
	for(size_t i = 0; i < 4; ++i){
		if(idNeighbors[i] != -1){
			fooNeigh.push_back(idNeighbors[i]);
		}
	}
	
	return(fooNeigh);
	
}


PatchAgent::idPatchNeighbors PatchAgent::findPatchNeighbors()
{
	idPatchNeighbors fooNeigh;
	
	int32_t idNeighbors[8];
	
	Environment::grid_t gridData = _myEnv->getGrid();
	
	idNeighbors[0] = getId() + gridData._quadX; //N
	idNeighbors[1] = getId() - gridData._quadX; //S
	idNeighbors[2] = getId() - 1; //W
	idNeighbors[3] = getId() + 1; //E
	
	idNeighbors[4] = idNeighbors[0]  - 1; //NW
	idNeighbors[5] = idNeighbors[0]  + 1; //NE
	idNeighbors[6] = idNeighbors[1]  - 1; //SW
	idNeighbors[7] = idNeighbors[1]  + 1; //SE
	
	uint32_t Xoffset = getId() % gridData._quadX;
	
	if(getId() >= gridData._quadX * (gridData._quadY - 1)){ //Sin vecinos N
		idNeighbors[0] = -1;
		idNeighbors[4] = -1;
		idNeighbors[5] = -1;
	}
	
	if(getId() >= 0 && getId() < gridData._quadX ){ // Sin vecinos S
		idNeighbors[1] = -1;
		idNeighbors[6] = -1;
		idNeighbors[7] = -1;
	}
	
	if(Xoffset == 0){ // Sin vecinos W
		idNeighbors[2] = -1;
		idNeighbors[4] = -1;
		idNeighbors[6] = -1;
	}
	
	if(Xoffset == (gridData._quadX - 1)){ // Sin vecinos E
		idNeighbors[3] = -1;
		idNeighbors[5] = -1;
		idNeighbors[7] = -1;
	}
	
	for(size_t i = 0; i < 8; ++i){
		if(idNeighbors[i] != -1){
			fooNeigh.push_back(idNeighbors[i]);
		}
	}
	
	return(fooNeigh);
	
}






