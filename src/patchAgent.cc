#include <patchAgent.hh>
#include <environment.hh>

std::shared_ptr<Environment> PatchAgent::_myEnv;


PatchAgent::PatchAgent(const uint32_t &_id)
{
	this->_id = _id;

}


PatchAgent::~PatchAgent(void)
{
	;
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