#include <patchAgent.hh>
#include <environment.hh>
#include <utils.hh>

std::shared_ptr<Environment> PatchAgent::_myEnv;


PatchAgent::PatchAgent(const uint32_t &id)
{
	static thread_local std::random_device device;
	static thread_local std::mt19937 rng(device());
	
	this->_id = id;
	
	
	Environment::grid_t gridData = _myEnv->getGrid();
	
	_myZone = nullptr;
	// Largo del cuadrante (en [m])         : ._quadSize;
	//Dimensiones del mapa                  : ._xMin, ._xMax, ._yMin, ._yMax;
	//Dimensiones del mapa                  : ._mapWidth, ._mapHeight;
	//Cantidad de cuadrantes en el eje X e Y: ._quadX, _quadY;
	
	
	_myQuad.idX = _id % gridData._quadX;
	_myQuad.idY = _id / gridData._quadX;
	
	_myQuad.width  = gridData._quadSize;
	_myQuad.height = gridData._quadSize;
	
	_myQuad.x0 = gridData._xMin + _myQuad.idX*_myQuad.width;
	_myQuad.y0 = gridData._yMin + _myQuad.idY*_myQuad.height;
	
	_myQuad.x1 = _myQuad.x0 + _myQuad.width;
	_myQuad.y1 = _myQuad.y0 + _myQuad.height;
	
	_myQuad.xc = _myQuad.x0 + _myQuad.width / 2;
	_myQuad.yc = _myQuad.y0 + _myQuad.height / 2;
	
	double h;
	_myEnv->getProjector().Reverse(_myQuad.xc, _myQuad.yc, 0, _myQuad.lat, _myQuad.lon, h);
	
	// Nivel de inundación del patch
	_levelFlood = 0; //m
	
	// Cuando un patchAgent pertence a una zona inundable
	// entonces este valor cambia al atributo 'maxLevelFlood' de
	// la zona respectiva
	_maxLevelFlood = 0; 
	
	//std::cout << _id << "\t: (" << _myQuad.x0 << ","<< _myQuad.y0 << ") -> (" << _myQuad.x1 << "," <<  _myQuad.y1 << ")" << std::endl;
	//std::cout << _id << std::fixed << std::setprecision(8) <<"\t: (" << _myQuad.xc << ","<< _myQuad.yc << ") -> (" << _myQuad.lat << "," <<  _myQuad.lon << ")" << std::endl;
	
	// Si el patch está dentro del area de movimiento de los agentes
	// se considera dentro de la ciudad
	if(_myQuad.xc >= gridData._xSimMin && _myQuad.xc <= gridData._xSimMax && _myQuad.yc >= gridData._ySimMin && _myQuad.yc <= gridData._ySimMax ){
		_isInCity = true;
		_myEnv->addPatchAgentInCity(this);
	}

	
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

/**
 * @brief Retorna datos geográficos del patch agent
 * @return 
 	struct quad_s{
		// Coordenadas (x,y) del vértice inferior izquierdo.
		double x0, y0;
		
		// Coordenadas (x,y) del vértice superior derecho.
		double x1, y1;
		
		// Coordenadas (x,y) del centro.
		double xc, yc;
		
		// Coordenadas (lat,lon) del centro.
		double lat, lon;
		
		// ancho y alto.
		double width, height;
		
		// identificadores numericos del cuadrante
		uint32_t idX, idY;
	}
 */
PatchAgent::quad_t PatchAgent::getQuadInfo()
{
	return(_myQuad);
}


void PatchAgent::evacMonitor(bool m)
{
	_isEvacMonitor = m;
}

bool PatchAgent::isEvacMonitor()
{
	return(_isEvacMonitor);
}

/*
void PatchAgent::setGradient(double g)
{
	_gradient = g;
}
	
double PatchAgent::getGradient()
{
	return(_gradient);
}
*/

bool PatchAgent::haveElevation()
{
	return(_haveElevation);
}
void PatchAgent::haveElevation(bool h)
{
	_haveElevation = h;
}

int32_t PatchAgent::getElevation()
{
	if(_elevation == -1){
		// Se debe determinar la elevación del partch a través del servidor de elevación.
		// Esto se hace sólo una vez.
		// La latitud y longitud del patch son las coordenadas del punto central de él.
		std::string req;
		req = global::params.elevationServer.URL + "/lookup?locations=";
		req += std::to_string(_myQuad.lat) + "," + std::to_string(_myQuad.lon);

		json geoInfoTest;
		utils::restClient_get(req, geoInfoTest);
		_elevation = geoInfoTest["results"][0]["elevation"].get<int>();
	}

	return(_elevation);
}

void PatchAgent::setElevation(int32_t elevation)
{
	_elevation = elevation;
}

bool PatchAgent::isInCity()
{
	return(_isInCity);
}

void PatchAgent::isInCity(bool c)
{
	_isInCity = c;
}

void PatchAgent::setProbDebris(double p)
{
	_probDebris = p;
}

double PatchAgent::getProbDebris()
{
	return(_probDebris);
}

bool PatchAgent::isDebrisFree()
{
	return(_isDebrisFree);
}

void PatchAgent::isDebrisFree(bool d)
{
	_isDebrisFree = d;
}


void PatchAgent::setZone(Zone* z)
{
	_myZone = z;
}
Zone* PatchAgent::getZone()
{
	return(_myZone);
}

void PatchAgent::isFloodable(bool m)
{
	_isFloodable = m;
}


bool PatchAgent::isFloodable()
{
	return(_isFloodable);
}

//////////////////////////////////
// Este método es llamado por
// void Environment::updateQuads()
void PatchAgent::updateLevelFlood()
{
	////////////////////////////////////////
	// Identificadores X,Y del patchAgent. 
	// _myQuad.idX 
	// _myQuad.idY 
	
	////////////////////////////////////////
	// Parámetros de la inundación.
	std::string floodDirection = _myEnv->getFloodParams().direction;
	double Vlevel = _myEnv->getFloodParams().speedWaterLevel;// el agua sube Vlevel m/s en cada cuadrante
	double Vprop  = _myEnv->getFloodParams().speedWaterProp; // el agua se propaga horizontalmente Vprop m/s
	
	////////////////////////////////////////
	// Solicitar los identificadores X,Y de 
	// los patchAgents que están más al N,S,W,E
	Zone::NSWEPatchAgentsAllZone_t idsPatchAgentsAllZone = _myEnv->getNSWEPatchAgentsAllZones();	
	uint32_t quadXMin = idsPatchAgentsAllZone[2]; //further West 
	uint32_t quadXMax = idsPatchAgentsAllZone[3]; //further East		
	uint32_t quadYMin = idsPatchAgentsAllZone[1]; //further South
	uint32_t quadYMax = idsPatchAgentsAllZone[0]; //further North
	
	////////////////////////////////////////
	// Determinar si el patchAgent está 
	// inundado
	
	uint32_t floodTime = global::currTimeSim * global::params.deltaT -  _myEnv->getFloodParams().arrivalTime;
	
	bool patchIsFlooded = false;
	if(floodDirection == "W->E"){
		// cantidad de cuadrantes al oeste
		uint32_t deltaXquads  = std::abs((int)_myQuad.idX - (int)quadXMin);
		double   deltaXmeters = (deltaXquads + 1.0) * _myEnv->getGrid()._quadSize; 
	
		if(deltaXmeters/Vprop <= floodTime){
			patchIsFlooded = true;
		}
	}
	else if(floodDirection == "N->S"){
		// cantidad de cuadrantes al norte
		uint32_t deltaYquads  = std::abs((int)_myQuad.idY - (int)quadYMax);
		double   deltaYmeters = (deltaYquads + 1.0) * _myEnv->getGrid()._quadSize; 
	
		if(deltaYmeters/Vprop <= floodTime){
			patchIsFlooded = true;
		}
	}
	else if(floodDirection == "N->S;W->E" || floodDirection == "W->E;N->S") {
		uint32_t deltaXquads = std::abs((int)_myQuad.idX - (int)quadXMin) + 1;
		uint32_t deltaYquads = std::abs((int)_myQuad.idY - (int)quadYMax) + 1;
		
		uint32_t deltaXmeters = deltaXquads * _myEnv->getGrid()._quadSize;
		uint32_t deltaYmeters = deltaYquads * _myEnv->getGrid()._quadSize;
		
		double XYmeters = std::hypot(deltaXmeters, deltaYmeters);
		
		if(XYmeters/Vprop <= floodTime){
			patchIsFlooded = true;
		}
		
	}
	else if(floodDirection == "S->N"){
		// cantidad de cuadrantes al sur
		uint32_t deltaYquads  = std::abs((int)_myQuad.idY - (int)quadYMin);
		double   deltaYmeters = (deltaYquads + 1.0) * _myEnv->getGrid()._quadSize; 
	
		if(deltaYmeters/Vprop <= floodTime){
			patchIsFlooded = true;
		}
	}
	else if(floodDirection == "E->W"){
		// cantidad de cuadrantes al este
		uint32_t deltaXquads  = std::abs((int)_myQuad.idX - (int)quadXMax);
		double   deltaXmeters = (deltaXquads + 1.0) * _myEnv->getGrid()._quadSize; 
	
		if(deltaXmeters/Vprop <= floodTime){
			patchIsFlooded = true;
		}
	}
	
	////////////////////////////////////////
	// Si el patchAgent está inundado,
	// entonces aumentar el nivel del agua
	// segun la velocidad vertical establecida
	
	if(patchIsFlooded){
		//std::srand(std::time(0));
		//double thr = std::rand() / static_cast<double>(RAND_MAX);
		
		/*if(_levelFlood >= _maxLevelFlood && _maxLevelFlood != -1 ){
			_levelFlood = _maxLevelFlood;
		}
		else{
			_levelFlood += Vlevel*global::params.deltaT;
		}*/
		
		if(_maxLevelFlood == -1){
			_levelFlood += Vlevel*global::params.deltaT;
		}
		else if(_levelFlood < _maxLevelFlood ){
			_levelFlood += Vlevel*global::params.deltaT;
		}
		else{
			_levelFlood = _maxLevelFlood;
		}
		/*
		else if(thr < 0.95){
			_levelFlood += Vlevel*global::params.deltaT;
		}
		else{
			_levelFlood -= Vlevel*global::params.deltaT;
		}
		*/
	}

}

/*bool PatchAgent::isFloodLevelUpdatable()
{
	return(_isFloodLevelUpdatable);
}
void PatchAgent::isFloodLevelUpdatable(bool i)
{
	_isFloodLevelUpdatable = i;
}*/


double PatchAgent::getLevelFlood()
{
	return(_levelFlood);
}

void PatchAgent::setMaxLevelFlood(double m)
{
	_maxLevelFlood = m;
}

double PatchAgent::getMaxLevelFlood()
{
	return(_maxLevelFlood);
}





