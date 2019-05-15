#ifndef _PATCHAGENT_HH_
#define _PATCHAGENT_HH_
#include <glob.hh>

class Environment;

class PatchAgent {

public:

	typedef std::vector<PatchAgent*> Neighbors;
	static std::shared_ptr<Environment> _myEnv;


private:
	mutable std::mutex _mtx;
	uint32_t _id;
	uint32_t _quad;

	std::vector<uint32_t> _agentsInPatch;

public:
	PatchAgent(void);
	PatchAgent(const uint32_t&);

	~PatchAgent(void);

	void addAgent(const uint32_t &idAgent);
	void delAgent(const uint32_t &idAgent);
	std::vector<uint32_t>& getAgents();
};


#endif
