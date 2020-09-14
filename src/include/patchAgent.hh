#ifndef _PATCHAGENT_HH_
#define _PATCHAGENT_HH_
#include <glob.hh>

class Environment;

class PatchAgent {

public:

	typedef std::vector<PatchAgent*> Neighbors;
	typedef std::vector<uint32_t> idPatchNeighbors;
	static std::shared_ptr<Environment> _myEnv;


private:
	mutable std::mutex _mtx;
	uint32_t _id;
	uint32_t _quad;

	std::vector<uint32_t> _agentsInPatch;
	std::vector<uint32_t> _neighborsAgents; //NEW

public:
	PatchAgent(void);
	PatchAgent(const uint32_t&);

	~PatchAgent(void);

	uint32_t getId();
	void addAgent(const uint32_t &idAgent);
	void delAgent(const uint32_t &idAgent);
	std::vector<uint32_t>& getAgents();
	
	std::vector<uint32_t>& getNeighborsAgents();
	idPatchNeighbors findPatchNeighbors4();
	idPatchNeighbors findPatchNeighbors();
};


#endif
