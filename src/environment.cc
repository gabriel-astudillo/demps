#include <environment.hh>
#include <simulator.hh>


Environment::Environment(void)
{
	;
}

Environment::Environment(std::vector<Agent*> vAgents)
{
	this->_vAgents = vAgents;
}

Environment::Environment(const Environment& env)
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
void Environment::setRouter(const std::string& map_osrm)
{
	_router =  Router(map_osrm);
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
* @param fmap_zone: GeoJson de los datos del mapa
* @return void
*/

void Environment::setReferencePoint(const json& fmap_zone)
{
	std::list<double> map_x;
	std::list<double> map_y;

	double xMin,yMin;

	// Caso especial: sólo un feature
	/*for(auto& point : fmap_zone["features"][0]["geometry"]["coordinates"][0]) {
		map_x.push_back( point[1] );
		map_y.push_back( point[0] );
	}*/
	
	// Caso general: múltiples features
	for(auto& feature : fmap_zone["features"]) {
		//std::cout << feature.dump(4) << std::endl;
		if(feature["properties"]["zoneType"] == "initial" || 
		   feature["properties"]["zoneType"] == "safe"    ||
		   feature["properties"]["zoneType"] == "flood" ){
			for(auto& point : feature["geometry"]["coordinates"][0]){
				map_x.push_back( point[1] );
				map_y.push_back( point[0] );
			}
		}
		
	}
	
	map_x.sort();
	map_y.sort();

	//Coordenadas min  x e y.
	xMin = map_x.front();
	yMin = map_y.front();
	
	double lon, lat;
	lon = yMin;
	lat = xMin;
	_reference_point = Point2D(lon, lat);

}

Point2D Environment::getReferencePoint()
{
	return(_reference_point);
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
//void Environment::setProjector()
void Environment::setProjector(const json& fmap_zone)
{
	setReferencePoint(fmap_zone);
	
	//LocalCartesian (real lat0, real lon0, real h0=0, const Geocentric &earth=Geocentric::WGS84())
	_projector = LocalCartesian(_reference_point.y(), _reference_point.x(), 0, Geocentric::WGS84());
}

LocalCartesian Environment::getProjector()
{
	return(_projector);
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
void Environment::setReferenceZones(const json& freference_zones)
{
	for(const auto& feature : freference_zones["features"]) {
		this->addReferenceZone(feature);
	}
}

void Environment::addReferenceZone(const json& freference_zone_feature)
{
	_reference_zones.push_back(Zone(freference_zone_feature));
}

/**
* @brief Crea las zonas iniciales
*
* Las zonas iniciales son las zones donde se van a
* ubicar los agentes al inicio de la simulación
*
* @param _finitial_zones: GeoJson que representa las zonas iniciales.
* @return void
*/
void Environment::setInitialZones(const json& finitial_zones)
{
	for(const auto& feature : finitial_zones["features"]) {
		this->addInitialZone(feature);
	}
}

void Environment::addInitialZone(const json& finitial_zone_feature)
{
	_initial_zones.push_back(Zone(finitial_zone_feature));
}

/**
* @brief Crea las zonas inundables
*
* Las zonas inundables son las zonas que se inundan
* en el transcurso de la simulación
*
* @param _flood_zones: GeoJson que representa las zonas inundables.
* @return void
*/

void Environment::addFloodZone(const json& flood_zone_feature)
{
	_flood_zones.push_back(Zone(flood_zone_feature));
}


/**
* @brief Crea las lineas de monitoreo.
*
* A partir de un objeto GeoJSON LineString,
* se marcan los patchagents que monitorean 
* la evacuación.
*
* @param lineMonitor_feature: GeoJson que representa un LineString.
* @return void
*/
void Environment::addLineMonitorZone(const json& lineMonitor_feature)
{
	std::vector<Point2D> pointsLine;
	std::string lineMonitorID;
	
	//std::cout << lineMonitor_feature.dump(4) << std::endl;
	
	for(auto& geoPoint : lineMonitor_feature["geometry"]["coordinates"]) {
		//std::cout << geoPoint.dump(4) << std::endl;
		double x,y,z,h;
		h = 0.0;
		///////////////////////
		//  Definición:
		//       https://geographiclib.sourceforge.io/html/classGeographicLib_1_1LocalCartesian.html
		this->getProjector().Forward(geoPoint[1],geoPoint[0],h,x,y,z);
		
		Point2D pNew = Point2D(x,y);
		pointsLine.push_back(pNew);	
	}

	lineMonitorID = lineMonitor_feature["properties"]["nameID"].get<std::string>();
	
	std::vector<PatchAgent*> patchAgentsInLine;
	patchAgentsInLine = this->getPatchAgentsLineMonitor(pointsLine[0], pointsLine[1]);
	
	// Todos los patchAgents que están en la línea geográfica especificada
	// forman un grupo cuyo identificador es 'lineMonitorID'
	for(const auto& pAgent : patchAgentsInLine){
		pAgent->evacMonitor(true);
		
		/*
		try{
			pAgent->setGradient(lineMonitor_feature["properties"]["gradient"].get<double>());
		}
		catch (json::exception &e) {
			pAgent->setGradient(0.0);
		}*/
		
		this->addMonitorPatchAgentGroup(pAgent, lineMonitorID, PatchAgent::typeMonitor::lineMonitor);
	}	
}

/**
* @brief Crea un punto de monitoreo.
*
* A partir de un objeto GeoJSON Point, se crea un punto de monitoreo 
* en el mapa. Un punto de monitoreo consiste en una vecindad de Moore
* alrededor de un punto especificado.
*
* @param pointMonitor_feature: GeoJson que representa un Point.
* @return void
*/
void Environment::addPointMonitorZone(const json& pointMonitor_feature)
{
	std::string pointMonitorID;
	json geoPoint;
	
	pointMonitorID = pointMonitor_feature["properties"]["nameID"].get<std::string>();
	geoPoint = pointMonitor_feature["geometry"]["coordinates"];
	
	double x,y,z,h;
	h = 0.0;
	this->getProjector().Forward(geoPoint[1],geoPoint[0],h,x,y,z);
		
	Point2D pointMonitor = Point2D(x,y);
	
	// Se buscan los agentes que están en una vecindad de Moore
	// del punto especificado.
	std::vector<PatchAgent*> patchAgentsInPointNeighborhood;
	patchAgentsInPointNeighborhood = this->getPatchAgentsPointMonitor(pointMonitor);
	
	// Todos los patchAgents que están en la vecindad de Moore del punto geográfico especificado
	// forman un grupo cuyo identificador es 'pointMonitorID'
	for(const auto& pAgent : patchAgentsInPointNeighborhood){
		pAgent->evacMonitor(true);
		this->addMonitorPatchAgentGroup(pAgent, pointMonitorID, PatchAgent::typeMonitor::pointMonitor);
	}

	
}

/**
* @brief Crea los cuadrantes del mapa de la simulación
*
* @param _fmap_zone: GeoJson que representa el mapa.
* @param quadSize  : tamaño de un lado del cuadrante, en metros.
* @return void
*/
void Environment::setGrid(const json &fmap_zone, uint32_t offset, uint32_t quadSize)
{
	std::list<double> map_x;
	std::list<double> map_y;
	
	// para buscar los cuadrantes relacionados con el
	// movimiento de los agentes
	std::list<double> map_xSim;
	std::list<double> map_ySim;

	/*for(auto& point : fmap_zone["features"][0]["geometry"]["coordinates"][0]) {
		double x,y,z;
		this->getProjector().Forward(point[1],point[0],0,x,y,z);

		map_x.push_back( x );
		map_y.push_back( y );
	}*/
	
	
	// Caso general: múltiples features
	for(auto& feature : fmap_zone["features"]) {
		//std::cout << feature.dump(4) << std::endl;
		if(feature["properties"]["zoneType"] == "initial" || feature["properties"]["zoneType"] == "safe"/* ||
		   feature["properties"]["zoneType"] == "flood" */){
			for(auto& point : feature["geometry"]["coordinates"][0]){
				double x,y,z;
				this->getProjector().Forward(point[1],point[0],0,x,y,z);

				map_x.push_back( x );
				map_y.push_back( y );
				
				map_xSim.push_back( x );
				map_ySim.push_back( y );
			}
		}
		
		if(feature["properties"]["zoneType"].get<std::string>() == "flood"){
			for(auto& point : feature["geometry"]["coordinates"][0]){
				double x,y,z;
				this->getProjector().Forward(point[1],point[0],0,x,y,z);

				map_x.push_back( x );
				map_y.push_back( y );
			}
		}
		
	}
	
	map_x.sort();
	map_y.sort();
	
	map_xSim.sort();
	map_ySim.sort();

	//Coordenadas min y max para x e y.
	_grid._xMin = map_x.front() - offset;
	_grid._xMax = map_x.back()  + offset;
	_grid._yMin = map_y.front() - offset;
	_grid._yMax = map_y.back()  + offset;
	
	//Coordenadas min y max para xSim e ySim.
	_grid._xSimMin = map_xSim.front();//- offset;
	_grid._xSimMax = map_xSim.back() ;//+ offset;
	_grid._ySimMin = map_ySim.front();//- offset;
	_grid._ySimMax = map_ySim.back() ;//+ offset;

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


/*void Environment::setElevationData(std::map<int32_t, std::tuple<double, double, int32_t> >& elevationData)
{
	_elevationData = elevationData;
}*/

/*int32_t Environment::getElevationDataPatchAgent(uint32_t idPatchAgent)
{
	return( std::get<2>(_elevationData[idPatchAgent]));
}*/

void Environment::showGrid()
{
	std::cout << "\x1B[0;37m";
	std::cout << "Grid description:" << std::endl; 
	std::cout << "\tquadSize:" << _grid._quadSize << std::endl; 
	std::cout << "\txMin:" << _grid._xMin  << ", xMax:" << _grid._xMax << "\n";
	std::cout << "\tyMin:" << _grid._yMin  << ", yMax:" << _grid._yMax << "\n";
	std::cout << "Map description:" << std::endl; 
	std::cout << "\tmapWidth: " << _grid._mapWidth << ", mapHeight:" << _grid._mapHeight << "\n";
	std::cout << "\tquadX:" << _grid._quadX << ", quadY:"  << _grid._quadY << std::endl;
	
	std::cout << "\tReference Point: " << std::fixed << std::setprecision(8);
	std::cout << this->getReferencePoint() << std::endl; 
	
	std::cout << "\x1B[0m" << std::endl;
}

void Environment::setFloodParams(const json& floodParams)
{
	_floodParams.enable               = floodParams["enable"].get<bool>();
	
	_floodParams.direction            = floodParams["direction"].get<std::string>();
	_floodParams.arrivalTime          = floodParams["arrivalTime"].get<int32_t>();
	_floodParams.sampleStateInterval  = floodParams["sampleStateInterval"].get<uint32_t>();
	_floodParams.speedWaterLevel      = floodParams["speedWaterLevel"].get<double>();
	_floodParams.speedWaterProp       = floodParams["speedWaterProp"].get<double>();
	_floodParams.criticalLevel        = floodParams["criticalLevel"].get<double>();
	_floodParams.minSpeedFactor       = floodParams["minSpeedFactor"].get<double>();
	
	_floodParams.imagesEnable         = floodParams["imagesEnable"].get<bool>();
	_floodParams.imagesDir            = floodParams["imagesDir"].get<std::string>();
	
	_floodParams.stateEnable         = floodParams["stateEnable"].get<bool>();
	_floodParams.stateDir            = floodParams["stateDir"].get<std::string>();
}

Environment::floodParams_t Environment::getFloodParams()
{
	return(_floodParams);
}


/**
 * @brief Asigna patch agents a las zonas de inundación
 *
 * Por cada patch que existe en la zona demarcada para la simulación,
 * se revisa si pertenece a una zona de inundación. Si el patch tiene
 * asociado varias zonas de inundación, entonces se asigna a la que 
 * tenga mayor nivel de inundación según la carta de inundación.
 *
 * @return (void)
 */
void Environment::assignPatchAgentsToFloodZones()
{
	Environment::grid_t gridData = this->getGrid();
	#pragma omp parallel for //num_threads(omp_get_max_threads())
	for(size_t y = 0; y < gridData._quadY; y++) {
		for(size_t x = 0; x < gridData._quadX; x++) {
			uint32_t idPatch = x + y * gridData._quadX;
			PatchAgent* pAgent = this->getPatchAgent(idPatch);
			PatchAgent::quad_t  qInfo = pAgent->getQuadInfo();
			
			Point2D fooPoint = Point2D(qInfo.xc, qInfo.yc);
			for(auto& z : this->getFloodZones()){
				if( z.pointIsInside(fooPoint,0) ){
					if(pAgent->getZone() != nullptr){
						/*uint32_t orderZoneOld = pAgent->getZone()->getOrder();
						uint32_t orderZoneCur = z.getOrder();
						
						if(orderZoneCur > orderZoneOld){
							// Sacar el id del patch Agent de la zona anterior
							pAgent->getZone()->deletePatchAgent(idPatch);
							
							// Agregar id del patch Agent a la zona
							z.addPatchAgent(idPatch);
					
							pAgent->setZone(&z);
							pAgent->isFloodable(true);
							pAgent->setMaxLevelFlood(z.getMaxLevelFlood());						
						}*/

						double maxLevelFloodPatch = pAgent->getMaxLevelFlood();
						maxLevelFloodPatch == -1 ? maxLevelFloodPatch = 1000 : maxLevelFloodPatch = maxLevelFloodPatch;

						double maxLevelFloodZone = z.getMaxLevelFlood();
						maxLevelFloodZone == -1 ? maxLevelFloodZone = 1000 : maxLevelFloodZone = maxLevelFloodZone;

						if( maxLevelFloodZone >  maxLevelFloodPatch){
							// Sacar el id del patch Agent de su zona inundable
							pAgent->getZone()->deletePatchAgent(idPatch);

							// Agregar id del patch Agent a la zona
							z.addPatchAgent(idPatch);

							pAgent->setZone(&z);
							pAgent->isFloodable(true);
							pAgent->setMaxLevelFlood( z.getMaxLevelFlood() );		

						}

					}
					else{
						// Agregar id del patch Agent a la zona
						z.addPatchAgent(idPatch);
					
						pAgent->setZone(&z);
						pAgent->isFloodable(true);
						pAgent->setMaxLevelFlood(z.getMaxLevelFlood());
					}
				}
				
			}
			
		}
	}
}

// La velocidad del agente especifica disminuye en un
// factor inversamente proporcional al nivel de inundación
// del patchAgent que lo contiene, hasta un factor
// determinado de la velocidad inicial 'disiredSpeed'
// especificada en el modelo de Fuerza social (estructura this->_SFM)
void Environment::setFloodVelocityOf(Agent* agent){
	uint32_t patchId = agent->getQuad();
	PatchAgent* pAgent = this->getPatchAgent(patchId);
	
	if( pAgent->getLevelFlood() <= this->getFloodParams().criticalLevel &&  pAgent->getLevelFlood() > 0){
		//std::lock_guard<std::mutex> l(_mtx);
		
		double speed = sqrt( agent->currVelocity().squared_length());
		double newSpeed;
		double minSpeed = this->getFloodParams().minSpeedFactor * agent->getSFMstruct().disiredSpeed;
		
		newSpeed  = speed * this->getFloodParams().criticalLevel;
		newSpeed -= pAgent->getLevelFlood() * (speed - minSpeed);
		newSpeed /= this->getFloodParams().criticalLevel;
		
		//std::cout << "newSpeed:" << newSpeed << " -> ";
		Transformation scale(CGAL::SCALING, newSpeed, speed);
		agent->currVelocity(scale(agent->currVelocity()));
		//std::cout << "currSpeed:" << sqrt(agent->currVelocity().squared_length() )<< std::endl;
	}	
}



void Environment::setNSWEPatchAgentsAllZones()
{
	
	std::list<double> quadXMinCandidates;
	std::list<double> quadXMaxCandidates;
	std::list<double> quadYMinCandidates;
	std::list<double> quadYMaxCandidates;
	
	for(auto& z : this->getFloodZones()){
		z.setNSWEPatchAgentsInZone();	
		
		Zone::NSWEPatchAgentsInZone_t idsPatchAgents = z.getNSWEPatchAgentsInZone();
		uint32_t quadXMin = idsPatchAgents[2]; //further West 
		uint32_t quadXMax = idsPatchAgents[3]; //further East		
		uint32_t quadYMin = idsPatchAgents[1]; //further South
		uint32_t quadYMax = idsPatchAgents[0]; //further North
		
		quadXMinCandidates.push_back(quadXMin);
		quadXMaxCandidates.push_back(quadXMax);
		quadYMinCandidates.push_back(quadYMin);
		quadYMaxCandidates.push_back(quadYMax);
		
	}
	
	quadXMinCandidates.sort();
	quadXMaxCandidates.sort();
	quadYMinCandidates.sort();
	quadYMaxCandidates.sort();
	
	_NSWEPatchAgentsAllZones[2] = quadXMinCandidates.front(); //most further West 
	_NSWEPatchAgentsAllZones[3] = quadXMaxCandidates.back();  //most further East	
	_NSWEPatchAgentsAllZones[1] = quadYMinCandidates.front();  //most further South
	_NSWEPatchAgentsAllZones[0] = quadYMaxCandidates.back();  //most further North
	
}

Zone::NSWEPatchAgentsAllZone_t Environment::getNSWEPatchAgentsAllZones()
{
	return(_NSWEPatchAgentsAllZones);
}

Zone& Environment::getInitialZone(uint32_t id)
{
	return(this->_initial_zones[id]);
}

std::vector<Zone>&  Environment::getInitialZones()
{
	return(this->_initial_zones);
}

Zone& Environment::getReferenceZone(uint32_t id)
{
	return(this->_reference_zones[id]);
}

std::vector<Zone>& Environment::getReferenceZones()
{
	return(this->_reference_zones);
}


Zone& Environment::getFloodZone(uint32_t id)
{
	return(this->_flood_zones[id]);
}

std::vector<Zone>& Environment::getFloodZones()
{
	return(this->_flood_zones);
}

void Environment::orderFloodZones()
{
	std::vector<Zone> orderFloodZones{_flood_zones};
	
	_flood_zones.clear();
	_flood_zones.resize(orderFloodZones.size());
	for(const auto& z: orderFloodZones){
		int32_t orderZone = z.getOrder();
		std::cout << orderZone << std::endl;
		_flood_zones[orderFloodZones.size()-orderZone] = z;
	}
	
}

////////////////////////////////////////////////////////
//
/*
  Retorna JSON con las características de las zonas
  iniciales y seguras

            "zonesInfo": {
                "initial_zones": {
                    "ZonaInicial1": {
                        "centroide": [lon, lat],
                        "requiv": [rw, rh]
                    },
                    ...
                },
                "reference_zones": {
                    "Zona1": {
                        "centroide":[lon, lat],
                        "requiv": [rw, rh]
                    },
                    ...
                }
            }
*/
json Environment::getZonesInfo()
{
	json zonesInfo, zonesIni, zonesRef;

	for(auto &fooZone : this->getInitialZones()) {
		double lon = fooZone.getCentroidWGS84().x();
		double lat = fooZone.getCentroidWGS84().y();
		double rw  = fooZone.getRequiv().x();
		double rh  = fooZone.getRequiv().y();
		zonesIni[fooZone.getNameID()] = { {"centroide", {lon, lat}},{"requiv", {rw, rh}} };
	}

	for(auto &fooZone : this->getReferenceZones()) {
		double lon = fooZone.getCentroidWGS84().x();
		double lat = fooZone.getCentroidWGS84().y();
		double rw  = fooZone.getRequiv().x();
		double rh  = fooZone.getRequiv().y();
		zonesRef[fooZone.getNameID()] = { {"centroide", {lon, lat}},{"requiv", {rw, rh}} };
	}
	
	zonesInfo["initial_zones"]  = zonesIni;
	zonesInfo["reference_zones"] = zonesRef;
	
	return(zonesInfo);
}

void Environment::getZonesInfo(json& out)
{
	out = getZonesInfo();
}


void Environment::addAgent(Agent* newAgent)
{
	this->_vAgents.push_back(newAgent);
}

void Environment::addPatchAgent(PatchAgent* newAgent) 
{
	this->_pAgents.push_back(newAgent);
}

void Environment::addPatchAgentInCity(PatchAgent* newAgent)
{
	this->_pAgentsInCity.push_back(newAgent);
}

void Environment::addPatchAgentInStreets(PatchAgent* newAgent)
{
	this->_pAgentsInStreets.push_back(newAgent);
}

//uint32_t Environment::getQuadId(Agent* a)
uint32_t Environment::getQuadId(Point2D position)
{
	uint32_t quadId;
	Environment::grid_t gridData = this->getGrid();
	
	//double posX = a->position()[0];
	//double posY = a->position()[1];
	
	double posX = position[0];
	double posY = position[1];

	quadId = (int) (posX - gridData._xMin) / gridData._quadSize +
	          (int)((posY - gridData._yMin) / gridData._quadSize) * gridData._quadX;
	
	if( quadId >= gridData._quadX * gridData._quadY ){
		std::cout << std::endl;
		std::cout << global::currTimeSim << "\t" << "****BUG****" << std::endl;
		std::cout << "****quadId="<< quadId << std::endl;
		std::cout << "****position="<< position << std::endl;
	}

	return(quadId);
}

Point2D  Environment::getQuadXY(Point2D position)
{
	Environment::grid_t gridData = this->getGrid();
	
	double posX = position[0];
	double posY = position[1];
	
	uint32_t xQuad = (int) (posX - gridData._xMin) / gridData._quadSize;
	uint32_t yQuad = (int) (posY - gridData._yMin) / gridData._quadSize;
	
	return(Point2D(xQuad, yQuad));
}

std::vector<PatchAgent*> Environment::getPatchAgentsLineMonitor(Point2D pointIni, Point2D pointEnd)
{
	std::vector<PatchAgent*> patchAgents;
	
	getPatchAgentsLineMonitor(pointIni, pointEnd, patchAgents);
	
	return(patchAgents);
}

void Environment::getPatchAgentsLineMonitor(Point2D pointIni, Point2D pointEnd, std::vector<PatchAgent*>& patchAgents)
{
	Environment::grid_t gridData = this->getGrid();
	
	// Determinar las coordenadas x,y del cuadrante
	// que contiene al punto de inicio y final de la
	// linea
	Point2D quadXYIni = this->getQuadXY(pointIni);
	Point2D quadXYEnd = this->getQuadXY(pointEnd);
	//std::cout << "quadXYIni: " << quadXYIni <<  std::endl;
	//std::cout << "quadXYEnd: " << quadXYEnd <<  std::endl;
	
	// Based on Bresenham's line algorithm
  	// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	
	int32_t deltaX = quadXYEnd.x() - quadXYIni.x();
	int32_t deltaY = quadXYEnd.y() - quadXYIni.y();
	
	int32_t dx = abs(deltaX);
	int32_t sx = deltaX > 0 ? 1 : -1;
	int32_t dy = -abs(deltaY);
	int32_t sy = deltaY > 0 ? 1 : -1;
	int32_t error = dx + dy;
	
	int xQuad = quadXYIni.x();
	int yQuad = quadXYIni.y();
	
	patchAgents.clear();
	while(true){
		// La variable quadId almacena el identificador
		// del cuadrante que está en la línea definida
		// en el mapa.
		uint32_t quadId = xQuad + yQuad * gridData._quadX;
		
		//std::cout << "xQuad=" << xQuad << ",\tyQuad=" << yQuad;
		//std::cout << "==> quadID=" << quadId;
		//std::cout << std::endl;
		
		patchAgents.push_back(this->getPatchAgent(quadId));
		
		if(xQuad == quadXYEnd.x() && yQuad == quadXYEnd.y()) break;

		int error2 = 2*error;

		if(error2 >= dy){
			if(xQuad == quadXYEnd.x()) break;
			error += dy;
			xQuad += sx;
		}

		if(error2 <= dx){
			if(yQuad == quadXYEnd.y()) break; 
			error += dx;
			yQuad += sy;
		}
	}

}

std::vector<PatchAgent*> Environment::getPatchAgentsPointMonitor(Point2D pointMonitor)
{
	std::vector<PatchAgent*> patchAgents;
	
	getPatchAgentsPointMonitor(pointMonitor, patchAgents);
	
	return(patchAgents);
}

void Environment::getPatchAgentsPointMonitor(Point2D pointMonitor, std::vector<PatchAgent*>& patchAgents)
{
	patchAgents.clear();
	
	// Determinar el identificador del
	// patchAgent que contiene el punto 'pointMonitor'
	uint32_t quadIdMonitor = this->getQuadId(pointMonitor);

	// A través del identificador, determinar
	// el puntero del patchAgent y agregarlo a la lista 'patchAgents'
	PatchAgent* centralMonitor;
	centralMonitor = this->getPatchAgent(quadIdMonitor);
	patchAgents.push_back(centralMonitor);
	
	// Determinar los identificadores de los patchAgents
	// que están en una vecindad de Moore alrededor del patchAgent central.
	PatchAgent::idPatchNeighbors pAgentsNeighbors;
	pAgentsNeighbors = centralMonitor->findPatchNeighbors();
	
	// Por cada identificador, agregar el respectivo puntero
	// del patchAgent a la lista 'patchAgents'
	for(const auto& pAgent : pAgentsNeighbors){
		patchAgents.push_back(this->getPatchAgent(pAgent));
	}
	
	
}

void Environment::addMonitorPatchAgentGroup(PatchAgent* patchAgent, std::string idGroup, PatchAgent::typeMonitor type)
{
	if(type == PatchAgent::typeMonitor::pointMonitor){
		_pointsMonitorGroup[idGroup].push_back(patchAgent);
	}else if(type == PatchAgent::typeMonitor::lineMonitor){
		_linesMonitorGroup[idGroup].push_back(patchAgent);
	}
}


PatchAgent::monitorGroup Environment::getMonitorPatchAgentGroups(PatchAgent::typeMonitor type)
{
	PatchAgent::monitorGroup patchAgentGroup;
	if(type == PatchAgent::typeMonitor::pointMonitor){
		patchAgentGroup = _pointsMonitorGroup;
	}else if(type == PatchAgent::typeMonitor::lineMonitor){
		patchAgentGroup = _linesMonitorGroup;
	}
	
	return(patchAgentGroup);
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
* @brief Retorna un vector con los agentes el Environment
*
* @param void
* @return std::vector<Agent*>
*/
std::vector<Agent*>& Environment::getAgents()
{
	return(_vAgents);
}

/**
* @brief Retorna un puntero a un patch agente determinado por el id del cuadrante
*
* @param void
* @return std::vector<Agent*>
*/
PatchAgent* Environment::getPatchAgent(uint32_t id)
{
	return(_pAgents[id]);
}

/**
* @brief Retorna un vector con los punteros a cada patchAgent del environment
*
* @param void
* @return std::vector<patchAgent*>
*/
std::vector<PatchAgent*> Environment::getPatchAgents()
{
	return(_pAgents);
}

std::vector<PatchAgent*> Environment::getPatchAgentsInCity()
{
	return(_pAgentsInCity);
}

std::vector<PatchAgent*> Environment::getPatchAgentsInStreets()
{
	return(_pAgentsInStreets);
}



/**
* @brief Configura los vecinos de un agente identificado por su id.
* Al agente identificado se le asignan todos los agentes vecinos que
* están dentro del radio 'distanceMax y que están vivos
*
* @param idAgent: entero que identifica el agente
* @param distanceMax: radio de la vecindad
* @param agentNeighbors: Vector con los agentes vecinos [out]
*/
//void Environment::setNeighborsOf(const uint32_t& idAgent,const double& distanceMax, Agent::Neighbors& agentNeighbors)
void Environment::setNeighborsOf(const uint32_t& idAgent,const double& distanceMax)
{
	//Agent::Neighbors agentNeighbors;
	
	std::vector<uint32_t> idsAgents;

	Agent* agent = this->getAgent(idAgent);

	//idsAgents = this->getPatchAgent( agent->getQuad() )->getAgents();
	idsAgents = this->getPatchAgent( agent->getQuad() )->getNeighborsAgents();

	
	agent->clearAgentNeighbors(); //<--
	
	//agentNeighbors.clear();

	for(auto& id : idsAgents) {
		if(id != idAgent) {
			Agent* neighbor;
			neighbor = this->getAgent(id);

			//agent->addCloseNeighbors(neighbor);

			if( neighbor == NULL || !neighbor->getEvacuationDataStruct().isAlive) {
				continue;
			}


			double dist = distance(agent, neighbor);
			if(  dist < distanceMax && dist > 0 ) {
				//agent->addCloseNeighbors(neighbor);
				
				//agentNeighbors.push_back(neighbor);
				agent->addAgentNeighbors(neighbor); //<--
			}

		}
	}

}

//
// Modelo de densidad
//
void Environment::setDensityParams(const json& densityParams)
{
	_densityParams.enable      = densityParams["enable"].get<bool>();
	_densityParams.minDensity  = densityParams["minDensity"].get<double>();
	_densityParams.maxDensity  = densityParams["maxDensity"].get<double>();
	_densityParams.minVelocity = densityParams["minVelocity"].get<double>();
}

Environment::densityParams_t Environment::getDensityParams()
{
	return(_densityParams);
}
		
void Environment::setDensityOf(Agent* agent)
{
	agent->setDensity(
		 (double)agent->getAgentNeighbors().size() / ( 3.14*(global::params.attractionRadius - agent->radius())*(global::params.attractionRadius - agent->radius()) )
			 );
}

void Environment::setDensityVelocityOf(Agent* agent)
{	
	Vector2D curVel = agent->currVelocity();
	double   curSpeed = sqrt( curVel.squared_length());
	
	if(curSpeed > 0){
		Vector2D newVelocity = Vector2D(0.0, 0.0);
		double   newSpeed = 0.0;
	
		double rho = agent->getDensity();
	
		Environment::densityParams_t densityParams = this->getDensityParams();
		
		double minSpeed = 0.0;
		minSpeed = densityParams.minVelocity;
		
		/*uint32_t patchId = agent->getQuad();
		PatchAgent* pAgent = this->getPatchAgent(patchId);
		if( pAgent->getLevelFlood() > 0){
			minSpeed = this->getFloodParams().minSpeedFactor * agent->getSFMstruct().disiredSpeed;
		}*/
		
		//minSpeed = this->getFloodParams().minSpeedFactor * agent->getSFMstruct().disiredSpeed;
		//minSpeed = std::min(minSpeed, densityParams.minVelocity);
	
		if(rho >= densityParams.maxDensity){
			// Si la densidad >= densidad máxima, ajusta la nueva velocidad a la velocidad mínima
			//newVelocity = curVel / curSpeed * densityParams.minVelocity;
			newVelocity = curVel / curSpeed * minSpeed;
		}
		else if(rho <= densityParams.minDensity){
			// Si la densidad <= densidad mínima, mantiene la velocidad
			// Si la velocidad actual es menor que la velocidad mínima, ajusta
			// la nueva velocidad a dicha velocidad
			/*if(curSpeed <= densityParams.minVelocity){
				newVelocity = curVel / curSpeed * densityParams.minVelocity;
			}else{
				newVelocity = curVel;
			}*/
			
			// Segunda opcion. Para el agente no vaya más rápido que la
			// velocidad impuesta por la inundación
			/*if(curSpeed <= minSpeed){
				newVelocity = curVel / curSpeed * minSpeed;
			}else{
				newVelocity = curVel;
			}*/
			
			newVelocity = curVel;
		}
		else{
			double m;
		
			/*if(curSpeed <= densityParams.minVelocity){
				curSpeed = densityParams.minVelocity;
			}*/
			
			// Segunda opción
			if(curSpeed <= minSpeed){
				curSpeed = minSpeed;
			}
						
			//m  = curSpeed - densityParams.minVelocity;
			m  = curSpeed - minSpeed;
			m /= densityParams.maxDensity - densityParams.minDensity;
		
			//newSpeed = (densityParams.maxDensity - rho) * m + densityParams.minVelocity;
			newSpeed = (densityParams.maxDensity - rho) * m + minSpeed;
			newVelocity = curVel / curSpeed * newSpeed;
		}
	
		agent->currVelocity(newVelocity);
	}
	
	

}

void Environment::setGradientVelocityOf(Agent* agent)
{	
	Vector2D newVelocity = Vector2D(0.0, 0.0);
	double   newSpeed = 0.0;
	
	//uint32_t patchId = agent->getQuad();
	//PatchAgent* pAgent = this->getPatchAgent(patchId);
	
	Vector2D curVel = agent->currVelocity();
	double curSpeed = sqrt( curVel.squared_length());
	double gradient = agent->getGradient();
	
	if(curSpeed > 0){
		// Hiking function
		// https://escholarship.org/content/qt05r820mz/qt05r820mz_noSplash_8f1f13a718ba4a0db0079773ffa4a7af.pdf
		double reductionCoeff = exp(-3.5 * std::abs(0.05 + gradient) );
		newSpeed = curSpeed * reductionCoeff;
	
		//double minSpeed = this->getFloodParams().minSpeedFactor * agent->getSFMstruct().disiredSpeed;
		//if(newSpeed < minSpeed){
		//	newSpeed = minSpeed;
		//}
		newVelocity = curVel / curSpeed * newSpeed;
		//std::cout << "curSpeed:" << curSpeed << ", newSpeed:" << newSpeed << std::endl;

		agent->currVelocity(newVelocity);
	}
}

/*
	Ajusta la velocidad según el porcentaje de escombros del patch
    donde se encuentra el agente.
	
	Si el nivel de escombros sobrepasa cierto nivel (o el coeficiente de
    reducción es menor que un umbral), el método indica que es necesario que el 
	agente busque una nueva ruta.
*/

bool Environment::setDebrisVelocityOf(Agent* agent)
{
	Vector2D newVelocity = Vector2D(0.0, 0.0);
	double   newSpeed = 0.0;
	
	Vector2D curVel = agent->currVelocity();
	double curSpeed = sqrt( curVel.squared_length());
	
	bool seekNewRoute = false;
	
	if(curSpeed > 0){
		// Obtener el patchId del agent
		uint32_t patchId = agent->getQuad();
		PatchAgent* pAgent = this->getPatchAgent(patchId);

		double coverageDebris = pAgent->getProbDebris();
		double reductionCoeff;
		// La probabilidad de tener escombros se toma como un
		// porcentahe de cobertura de escombros del patch y se 
		// aplica la formula de:
		
		// 1) Lu, X., Yang, Z., Cimellaro, G. P., & Xu, Z. (2019). 
		// Pedestrian evacuation simulation under the scenario with earthquake-induced falling debris. Safety science, 114, 61-71.
		// Fórmula presentada en pp. 67.
		//reductionCoeff = -8.39*exp(11.86*coverageDebris - 5.03) + 1.06;
		
		// 2) Amini, M., Sanderson, D. R., Cox, D. T., Barbosa, A. R., & Rosenheim, N. (2023). 
		// Methodology to incorporate seismic damage and debris to evaluate strategies to reduce life safety risk for 
		// multi-hazard earthquake and tsunami. Natural Hazards, 1-36.
		// Fórmula presentada en pp.20, Fig.10.
		reductionCoeff = -3.34*coverageDebris + 1;
		
		
		if(reductionCoeff <= 0.20){
			reductionCoeff = 0.20;
			seekNewRoute = true;
			
		}
		
		newSpeed = curSpeed * reductionCoeff;
		newVelocity = curVel / curSpeed * newSpeed;
		
		agent->currVelocity(newVelocity);
	}
		
	return(seekNewRoute);
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
		if(global::execOptions.showProgressBar) {
			pg.update(i);
		}

		Agent* agent = this->getAgent(i);

		auto response = this->getRouter()->route(agent->position(),global::params.randomWalkwayRadius);
		agent->_route = response.path();
	}

	if(global::execOptions.showProgressBar) {
		std::cout << std::endl;
	}

	//
	// AJUSTE: PASO 2
	//
	std::cout << "...(2/2)" << std::endl;
	pg.start(calibrationTime-1);
	for(uint32_t t = 0; t < calibrationTime; t++) {
		if(global::execOptions.showProgressBar) {
			pg.update(t);
		}

		#pragma omp parallel for
		for(uint32_t i = 0; i < this->getTotalAgents(); i++) {
			Agent* agent = this->getAgent(i);

			if(agent->_route.empty()) {
				auto response = this->getRouter()->route(agent->position(),global::params.randomWalkwayRadius);
				agent->_route = response.path();
			}
			agent->randomWalkwayForAdjustInitialPosition();
		}
	}
	
	//
	// Despues que se posicionan en las calles, 
	// ajustar velocidad de los agentes a cero. (reposo)
	//
	#pragma omp parallel for
	for(uint32_t i = 0; i < this->getTotalAgents(); i++) {
		Agent* agent = this->getAgent(i);

		agent->currVelocity(Vector2D(0.0,0.0));
	}
}

void Environment::determinatePAgentsInStreets()
{
	/*
	if(global::params.elevationPatchDataValid && this->haveElevation()){
		// Cambiar el atributo _elevation con los datos de elevación del archivo externo
		_elevation = _myEnv->getElevationDataPatchAgent(id);
		
		// si la elevacion > 0 , se asume que el patch está dentro de la ciudad
		if(_elevation > 0) {
			_myEnv->addPatchAgentInCity(this);
			_isInCity = true;
			std::uniform_real_distribution<double> unif(0.0, 1.0);
			_probDebris = unif(rng);
		}
	}
	*/
	
	ProgressBar pg;
	pg.start(this->getPatchAgentsInCity().size()-1);
	
	int32_t i=0;
	for(const auto& pAgent : this->getPatchAgentsInCity()){
		if(global::execOptions.showProgressBar) {
			pg.update(i++);
		}
		
		int32_t totalAgentsInPatch = pAgent->getAgents().size();
		if(totalAgentsInPatch > 0){
			this->addPatchAgentInStreets(pAgent);
		}
	}
	
}

void Environment::determinatePAgentsWithDebris(double debrisRatio, int& pAgentsWithDebris)
{
	std::random_device device;
	std::mt19937 rng(device());
	
	pAgentsWithDebris = 0; 
	
	ProgressBar pg;
	pg.start(this->getPatchAgentsInStreets().size()-1);
	
	int i = 0;
	for(const auto& pAgent : this->getPatchAgentsInStreets()){
		if(global::execOptions.showProgressBar) {
			pg.update(i++);
		}
		
		std::uniform_real_distribution<double> unif(0.0, 1.0);
		double debrisTrigger = unif(rng);

		if( debrisTrigger < debrisRatio){
			pAgent->isDebrisFree(false); 
			pAgentsWithDebris++;
			
			
			std::uniform_real_distribution<double> unif(0.0, 1.0);
			pAgent->setProbDebris(unif(rng));
			
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
		if(global::execOptions.showProgressBar) {
			pg.update(i);
		}

		Agent* agent = this->getAgent(i);
		
		//Ajustar la primera vez que utlizará el teléfono
		agent->setNextTimeUsePhone();
		
		//A todos los agentes se les asigna datos de su zona segura
		//El metodo setSafeZoneAttribAgent() discrimina si el dato
		//se puede utilizar por el agente (residente) o no (visitante II)
		//Para los visitantes II, los datos de la zona segura se utilizan
		//para determinar algunas variables de salida (distancia a la zona segura
		//mas cercana, etc)
		if(agent->safeZone() == nullptr){ //El agente no tiene su zona segura asignada				
			this->setSafeZoneAttribAgent(agent);

			if( !agent->safeZoneDataIsFake() ){
				auto response = this->getRouter()->route(agent->position(),agent->getTargetPos());
				agent->_route = response.path();
			}
			
		}

	}
}

void Environment::setSafeZoneAttribAgent(Agent* agent)
{			
	double distance = DBL_MAX;
	Point2D  fooTarget;
	std::string safeZoneNameID;
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
		safeZoneNameID = reference_zone.getNameID();
		double fooDistance = sqrt(CGAL::squared_distance(agent->position(), fooTarget));

		if( fooDistance < distance ) {
			distance = fooDistance;			
			agent->setTargetPos(fooTarget);
			agent->distanceToTargetPos(distance);
			agent->setSafeZoneID(safeZoneNameID);
			agent->safeZone(&reference_zone);
			
			//Los datos de la zona segura son falsos
			//solo para los visitantes tipo II
			//Para los visitantes II, los datos de la zona segura se utilizan
			//para determinar algunas variables de salida (distancia a la zona segura
			//mas cercana, etc).
			//Si durante la simulación logran determinar datos de zona segura a través 
			//de otros agentes cuyos
			//datos sean válidos, por transitividad consideran válidos
			if(agent->model() == Visitors_II){
				agent->safeZoneDataIsFake(true);
			}								
			else{
				agent->safeZoneDataIsFake(false);
			}					
		}
	}//Fin for
	
	//El agente pertenece a los agentes designados
	//de la zona asignada.
	agent->safeZone()->addAgentAssigned(agent->id());
	
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
		
		if(!agent->isAlive()){
			continue;
		}
		
		std::string fooSafeZoneNameID = agent->getSafeZoneID();
		
		if(fooSafeZoneNameID == "NA"){
			agent->inSafeZone(false);	
		}
		else{
			auto reference_zone = agent->safeZone();
			
			bool isInside = reference_zone->pointIsInside(agent->position(), global::params.closeEnough);
			
			//Si el agente está dentro de su Zona Segura
			if(isInside) {
				//Si el agente no está marcado como "en zona segura"
				if(!agent->inSafeZone()){
					agent->inSafeZone(true);
					agent->isMoving(false);
					agent->isWaiting(false);
					agent->evacuationTime(global::currTimeSim);

					reference_zone->addAgent(agent->id());
					reference_zone->updateAgentsDensity();
				}
				
				double distToTargetPoint = sqrt(CGAL::squared_distance(agent->position(), agent->getTargetPos()));
				if( distToTargetPoint < 5){
					Agent::evacuationData_t eData = agent->getEvacuationDataStruct();
					eData.arrivedAtDestinationPoint = true;
					agent->setEvacuationDataStruct(eData);
					
					//agent->evacuationTime(global::currTimeSim);
				}
			} 
			
			//else {
				//agent->inSafeZone(false);				
			//}	
		}
	}
}

/*void Environment::enableFloodLevelUpdate()
{
	#pragma omp parallel for
	for(uint32_t zId = 0; zId < this->getFloodZones().size(); zId++ ){
		// por cada zona inundable, rescata los patch agents respectivos
		Zone floodZone = this->getFloodZone(zId);
	
		for(const auto& pAgentID : floodZone.patchAgentsInZone()){
		
			PatchAgent* pAgent = this->getPatchAgent(pAgentID);
			pAgent->isFloodLevelUpdatable(true);
		}
	
	}
}*/

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
		
		/*
		// Esta idea no sirve para mostrar el mapa de inundación. 
		//Los agentes andan por las calles y a medida
		// que avanzan, van dejando patchAgents vacíos, lo que implica que 
		// estos actualizan el nivel de inundación
		// Por otro lado, hay que evaluar si los resultados son equivalente
		// al metodo de abajo. Podrían existir ambos, uno acelerado que no
		// involucre visualización, y otro lento.
		PatchAgent* pAgent = this->getPatchAgent(agent->getQuad());
		if(this->getFloodParams().enable && pAgent->isFloodable() && pAgent->isFloodLevelUpdatable()){
			//std::cout << "idPatchAgent: " << pAgent->getId() << std::endl;
			pAgent->updateLevelFlood();
			pAgent->isFloodLevelUpdatable(false);
			//std::cout << "Level Flood: " << pAgent->getLevelFlood() << std::endl;
		}
		*/
		
	}
	
	if(this->getFloodParams().enable && global::currTimeSim*global::params.deltaT >= this->getFloodParams().arrivalTime ){
		#pragma omp parallel for 
		for(uint32_t zId = 0; zId < this->getFloodZones().size(); zId++ ){
			// por cada zona inundable, rescata los patch agents respectivos
			Zone floodZone = this->getFloodZone(zId);
		
			for(const auto& pAgentID : floodZone.patchAgentsInZone()){
			
				PatchAgent* pAgent = this->getPatchAgent(pAgentID);
				pAgent->updateLevelFlood();
			}
		
		}
	}
	
}

/**
* @brief Actualiza las estadisticas de la simulacion
*
* Actualiza las variables que se utilizan para entregar los datos
* de salida del simulador. Este método se llama después de updateQuads().
* Variables que se actualizan:
*     global::simOutputs.logs.usePhone
*     global::simOutputs.logs.velocity
*     global::simOutputs.logs.deceasedAgents
*     global::simOutputs.logs.SIRpanic
*     global::simOutputs.logs.zonesDensity
*
* @param void
* @return void
*/
void Environment::updateLogsStats()
{
	uint32_t totalAgents = this->getTotalAgents();
	double SIR[3] = {0.0, 0.0, 0.0};
	uint32_t deceasedAgents = 0;	
	uint32_t inSafeZonesAgents = 0;	
	uint32_t movingAgents = 0;
	uint32_t waitingAgents = 0;

	//#pragma omp parallel for //schedule (dynamic,8)
	for(uint32_t i = 0; i < totalAgents; i++) {
		Agent* agent = this->getAgent(i);
		
		global::simOutputs.logs.usePhone[global::currTimeSim] += agent->getUsingPhone();
		global::simOutputs.logs.velocity[agent->id()] += std::to_string(sqrt(agent->currVelocity().squared_length())) + ":";
		
		Agent::evacuationData_t agEvac = agent->getEvacuationDataStruct();
		
		deceasedAgents += !agEvac.isAlive;
		inSafeZonesAgents += agEvac.inSafeZone;
		movingAgents += agEvac.isMoving;
		waitingAgents += agEvac.isWaiting;
		
		Agent::panic_t agPanic = agent->getPanicStruct();
		
		// El modelo de infección sólo tiene sentido
		// si el agente está vivo.
		if(agent->getEvacuationDataStruct().isAlive){
			SIR[agPanic.stateCode]++;
		}
	}
	
	std::ostringstream statsDeceasedAgents;
	statsDeceasedAgents << global::currTimeSim <<  ":" ;
	statsDeceasedAgents << inSafeZonesAgents <<  ":" ;
	statsDeceasedAgents << movingAgents <<  ":" ;
	statsDeceasedAgents << deceasedAgents <<  ":" ;
	statsDeceasedAgents << waitingAgents <<  ":" ;
	statsDeceasedAgents << (double)inSafeZonesAgents/totalAgents <<  ":" ;
	statsDeceasedAgents << (double)movingAgents/totalAgents <<  ":" ;
	statsDeceasedAgents << (double)deceasedAgents/totalAgents <<  ":" ;
	statsDeceasedAgents << (double)waitingAgents/totalAgents;
	global::simOutputs.logs.deceasedAgents.push_back(statsDeceasedAgents.str());
	
	std::ostringstream statsSIR;
	statsSIR << global::currTimeSim <<  ":" ;
	statsSIR << SIR[Agent::panicState.susceptible] <<  ":" ;
	statsSIR << SIR[Agent::panicState.infected] <<  ":" ;
	statsSIR << SIR[Agent::panicState.recovered]  <<  ":" ;
	statsSIR << std::fixed << std::setprecision(10);
	statsSIR << SIR[Agent::panicState.susceptible]/totalAgents <<  ":" ;
	statsSIR << SIR[Agent::panicState.infected]/totalAgents <<  ":" ;
	statsSIR << SIR[Agent::panicState.recovered]/totalAgents;
	global::simOutputs.logs.SIRpanic.push_back(statsSIR.str());


	std::string logString;
	
	logString = std::to_string(global::currTimeSim*global::params.deltaT);
	
	for(auto& reference_zone : this->getReferenceZones() ) {	
		logString +=  ":" + reference_zone.getNameID() + ":" +  \
		            std::to_string(reference_zone.getTotalAgents()) + ":" + \
		            std::to_string(reference_zone.getAgentsDensity());		
	}
	global::simOutputs.logs.zonesDensity.push_back(logString);
	
}


double Environment::distance(Agent* a, Agent* b)
{
	return(sqrt(CGAL::squared_distance(a->position(),b->position())) - a->radius() - b->radius()  );
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
