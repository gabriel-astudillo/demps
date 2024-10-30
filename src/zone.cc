#include <zone.hh>
#include <environment.hh>

std::shared_ptr<Environment> Zone::_myEnv;
//std::shared_ptr<Environment> ZoneBasic::_myEnv;

/*
ZoneBasic::ZoneBasic()
{
	_projector = _myEnv->getProjector();
}

void ZoneBasic::setCentroidWGS84(double lon, double lat)
{
	double x,y,z,h;
	h = 0.0;
	_projector.Forward(lat, lon, h, x, y, z);
	
	_centroidWSG84 = Point2D(lon,lat);
	_centroid      = Point2D(x,y);
}

void ZoneBasic::setRequiv(double rw, double rh)
{
	_rEquiv = Vector2D(rw, rh);
}

Point2D ZoneBasic::getCentroidWGS84()
{
	return(_centroidWSG84);
}

Vector2D ZoneBasic::getRequiv()
{
	return(_rEquiv);
}
*/

////////////////////////////////////////////////////////////////////////////////
//
//

Zone::Zone(void)
{
	;
}
//Zone::Zone(const json &_freference_point,const json &_fpolygon)
Zone::Zone(const json &feature)
{	
	_projector = _myEnv->getProjector();

	if(feature["geometry"]["type"]!="Polygon") {
		std::cerr << "Error::input feature is not a polygon" << std::endl;
		exit(EXIT_FAILURE);
	}

	_nameID   = feature["properties"]["nameID"].get<std::string>();
	_zoneType = feature["properties"]["zoneType"].get<std::string>();
	
	
	
	if(_zoneType == "flood"){
		_maxLevelFlood = feature["properties"]["maxLevelFlood"].get<double>();
		
		_order    = feature["properties"]["order"].get<int32_t>();
	}
	else{
		_maxLevelFlood = 0;
		_order = -1;
	}
	
	_xyMin = Point2D(50000, 50000);
	_xyMax = Point2D(-50000, -50000);
	for(auto& fpoint : feature["geometry"]["coordinates"][0]) {
		double x,y,z,h;

		h = 0.0;
		///////////////////////
		//  Definición:
		//       https://geographiclib.sourceforge.io/html/classGeographicLib_1_1LocalCartesian.html
		_projector.Forward(fpoint[1],fpoint[0],h,x,y,z);
		
		Point2D pNew = Point2D(x,y);
		
		_polygon.push_back(pNew);
		
		//Determinar xMin,xMax, yMin, yMax
		double xMin, xMax;
		xMin = std::min(_xyMin[0], pNew[0]);
		xMax = std::max(_xyMax[0], pNew[0]);
		
		double yMin, yMax;
		yMin = std::min(_xyMin[1], pNew[1]);
		yMax = std::max(_xyMax[1], pNew[1]);
		
		_xyMin = Point2D(xMin, yMin);
		_xyMax = Point2D(xMax, yMax);
			
	}
	
	_area          = _polygon.area();
	_agentsDensity = 0.0;

	// Insert the polygons into a constrained triangulation
	_cdt.insert_constraint(_polygon.vertices_begin(), _polygon.vertices_end(), true);

	// Mark facets that are inside the domain bounded by the polygon
	// Ref: https://doc.cgal.org/latest/Triangulation_2/Triangulation_2_2polygon_triangulation_8cpp-example.html
	mark_domains(_cdt);
	
	_avgCentroidX = AvgDym();
	_avgCentroidY = AvgDym();
	
	//Determinar el factor de forma de la zona
	_shapeForm = (double)(_xyMax.y() - _xyMin.y()) / (double)(_xyMax.x() - _xyMin.x()) ;
	
	// Calcular los semi-lados del rectangulo equivalente de la zona
	double rw, rh;
	rw = 0.5*sqrt(_area/_shapeForm);
	rh = 0.5*sqrt(_area*_shapeForm);
	_rEquiv = Vector2D(rw,rh);
	
}

Zone::Zone(const Zone &_z)
{
	this->_cdt     = _z._cdt;
	this->_nameID  = _z._nameID;
	this->_zoneType = _z._zoneType;
	this->_maxLevelFlood = _z._maxLevelFlood;
	this->_order         = _z._order;
	this->_polygon = _z._polygon;
	this->_area    = _z._area;
	this->_projector     = _z._projector;
	this->_agentsDensity = _z._agentsDensity;
	this->_agentsInZone  = _z._agentsInZone;
	this->_patchAgentsInZone = _z._patchAgentsInZone;
	this->_centroid  = _z._centroid;
	this->_xyMin  = _z._xyMin;
	this->_xyMax  = _z._xyMax;
	this->_avgCentroidX = _z._avgCentroidX;
	this->_avgCentroidY = _z._avgCentroidY;
	this->_shapeForm    = _z._shapeForm;
	this->_rEquiv       = _z._rEquiv;
}

Zone::~Zone(void)
{
	;
}

Zone& Zone::operator=(const Zone &_z)
{
	this->_cdt     = _z._cdt;
	this->_nameID  = _z._nameID;
	this->_zoneType = _z._zoneType;
	this->_maxLevelFlood = _z._maxLevelFlood;
	this->_order         = _z._order;
	this->_polygon = _z._polygon;
	this->_area    = _z._area;
	this->_projector     = _z._projector;
	this->_agentsDensity = _z._agentsDensity;
	this->_agentsInZone  = _z._agentsInZone;
	this->_patchAgentsInZone = _z._patchAgentsInZone;
	this->_xyMin  = _z._xyMin;
	this->_xyMax  = _z._xyMax;
	this->_avgCentroidX = _z._avgCentroidX;
	this->_avgCentroidY = _z._avgCentroidY;
	this->_shapeForm    = _z._shapeForm;
	this->_rEquiv       = _z._rEquiv;
	return(*this);
}

bool Zone::pointIsInside(const Point2D& testPoint, const double& bias)
{	
	CGAL::Bounded_side bside = CGAL::bounded_side_2(this->_polygon.vertices_begin(), this->_polygon.vertices_end(), testPoint, K() );
	if (bside == CGAL::ON_BOUNDED_SIDE || bside == CGAL::ON_BOUNDARY) {
		return(true);
	} 
	
	
	for(double k = 1.0; k > 0; k -= 0.5){
		std::vector<Point2D> testPoints = {
			testPoint + Vector2D(bias*k, 0.0),
			testPoint + Vector2D(bias*k, bias*k),
			testPoint + Vector2D(bias*k, bias*k),
			testPoint + Vector2D(0.0, bias*k),
			testPoint + Vector2D(-bias*k, bias*k),
			testPoint + Vector2D(-bias*k, 0.0),
			testPoint + Vector2D(-bias*k, -bias*k),
			testPoint + Vector2D(0.0, -bias*k)
		};
	
		for(const auto& tPoint: testPoints){
			bside = CGAL::bounded_side_2(this->_polygon.vertices_begin(), this->_polygon.vertices_end(), tPoint, K() );
			if (bside == CGAL::ON_BOUNDED_SIDE || bside == CGAL::ON_BOUNDARY) {
				return(true);
			}
		}
	}
	
	return(false);
}

////////////////////////////////////////////////
// Operaciones con _agentsInZone
// Estos son los agentes que están dentro de la 
// zona segura.
//
void Zone::addAgent(const uint32_t& idAgent)
{
	//#pragma omp critical
	{
		_agentsInZone.insert(idAgent);
	}
}

void Zone::deleteAgent(const uint32_t& idAgent)
{
	//#pragma omp critical
	{
		_agentsInZone.erase(idAgent);
	}
}

const std::set<uint32_t>& Zone::agentsInZone(void)
{
	return(_agentsInZone);
}

////////////////////////////////////////////////
// Operaciones con _agentsAssignedInZone
// Estos son los agentes que están asignados
// a la zona segura.
//
void Zone::addAgentAssigned(const uint32_t& idAgent)
{
	#pragma omp critical
	{
		_agentsAssignedInZone.insert(idAgent);
	}
	
}

void Zone::deleteAgentAssigned(const uint32_t& idAgent)
{
	#pragma omp critical
	{
		_agentsAssignedInZone.erase(idAgent);
	}
}

const std::set<uint32_t>& Zone::agentsAssignedInZone(void)
{
	return(_agentsAssignedInZone);
}

void Zone::updateAgentsDensity(void)
{
	//#pragma omp critical
	{
		_agentsDensity = _agentsInZone.size() / _area;
	}

}

uint32_t Zone::getTotalAgents(void)
{
	return(_agentsInZone.size());
}

double Zone::getAgentsDensity(void)
{
	return(_agentsDensity);
}

uint32_t Zone::getTotalAgentsAssigned(void)
{
	return(_agentsAssignedInZone.size());
}

std::string Zone::getNameID(void) const
{
	return(_nameID);
}

std::string Zone::getZoneType() const
{
	return(_zoneType);
}

double Zone::getMaxLevelFlood() const
{
	return(_maxLevelFlood);
}

int32_t Zone::getOrder() const
{
	return(_order);
}

Point2D Zone::generate(void)
{
	std::vector<Point2D> points;

	/*
		El CDT está constituido por "faces", que son triangulos que están delimitados
		por el contorno del polígono "_polygon".

		Por cada "faces", se toma el respectivo triangulo y se crean puntos aleatorios
		dentro de él.

		Luego, se escoge uno de ellos y se agrega a la coleccion de posibles puntos
		del agente.

		Al finalizar, se aleatorizan los puntos y se selecciona uno de ellos.
	*/
	
	double areaTotal = 0.0;
	for (CDT::Finite_faces_iterator fit=_cdt.finite_faces_begin(); fit!=_cdt.finite_faces_end(); ++fit) {
		if ( fit->info().in_domain() ) {
			// Obterner el triángulo del respectivo faces
			Triangle2D triangleFace = _cdt.triangle(fit) ;
			
			areaTotal += triangleFace.area();
		}
	}
	
	for (CDT::Finite_faces_iterator fit=_cdt.finite_faces_begin(); fit!=_cdt.finite_faces_end(); ++fit) {
		if ( fit->info().in_domain() ) {
			std::vector<Point2D> trianglePoints;

			// Obterner el triángulo del respectivo faces
			Triangle2D triangleFace = _cdt.triangle(fit) ;
			double triangleArea = triangleFace.area();
			double ratioAreas = triangleArea / areaTotal;

			// Crear un generador de puntos aleatorios dentro del triangulo "triangleFace"
			// Ref: https://doc.cgal.org/latest/Generator/Generator_2random_points_triangle_2_8cpp-example.html
			CGAL::Random_points_in_triangle_2<Point2D> generator( triangleFace );

			// Generar 100 puntos y almacenarlos en "trianglePoints"
			CGAL::cpp11::copy_n(generator,100,std::back_inserter(trianglePoints));

			// Aleatorizar la posición de los puntos dentro del vector "trianglePoints"
			std::random_shuffle(trianglePoints.begin(),trianglePoints.end());

			// Seleccionar el primero y colocarlo en el vector "points"
			//points.push_back(trianglePoints[0]);
			
			// Agregar puntos según el porcentaje de area del triangula
			// con respecto al area total.
			points.insert(points.end(), trianglePoints.begin(), std::next(trianglePoints.begin(), 100*ratioAreas));
			
			
			// Con ese nuevo punto, recalcular el centroide de la zona
			double avgCentroideX = _avgCentroidX(trianglePoints[0][0]);
			double avgCentroideY = _avgCentroidY(trianglePoints[0][1]);
		
			_centroid = Point2D(avgCentroideX, avgCentroideY);		
		}
	}
	
	/*
	// Calcular los semi-lados del rectangulo equivalente de la zona
	double rw, rh;
	rw = 0.5*sqrt(_area/_shapeForm);
	rh = 0.5*sqrt(_area*_shapeForm);
	_rEquiv = Vector2D(rw,rh);
	*/

	// Aleatorizar la posición de los puntos dentro del vector "points"
	std::random_shuffle(points.begin(),points.end());

	//std::cout << "points.size(): " << points.size() << std::endl;
	
	// Seleccionar el primero
	return(points[1]);
}

double Zone::getArea()
{
	return _area;
}

Point2D Zone::getCentroid()
{
	return(_centroid);
}

Point2D Zone::getCentroidWGS84()
{
	double latitude,longitude,h;
	_myEnv->getProjector().Reverse(_centroid[0],_centroid[1],0,latitude,longitude,h);
	
	return(Point2D(longitude, latitude));
}

Point2D Zone::getXYmin()
{
	return(_xyMin);
}

Point2D Zone::getXYminWGS84()
{
	double latitude,longitude,h;
	_myEnv->getProjector().Reverse(_xyMin[0],_xyMin[1],0,latitude,longitude,h);
	
	return(Point2D(longitude, latitude));
}

Point2D Zone::getXYmax()
{
	return(_xyMax);
}

Point2D Zone::getXYmaxWGS84()
{
	double latitude,longitude,h;
	_myEnv->getProjector().Reverse(_xyMax[0],_xyMax[1],0,latitude,longitude,h);
	
	return(Point2D(longitude, latitude));
}

double Zone::getshapeForm()
{
	return(_shapeForm);
}

Vector2D Zone::getRequiv()
{
	return(_rEquiv);
}

void Zone::addPatchAgent(uint32_t patchID)
{
	std::lock_guard<std::mutex> l(_mtx);
	_patchAgentsInZone.insert(patchID);
}

void Zone::deletePatchAgent(const uint32_t& patchID)
{
	std::lock_guard<std::mutex> l(_mtx);
	_patchAgentsInZone.erase(patchID);

}

/*
void Zone::assignPatchAgents()
{
	std::vector<Point2D> points;
	
	for (CDT::Finite_faces_iterator fit=_cdt.finite_faces_begin(); fit!=_cdt.finite_faces_end(); ++fit) {
		if ( fit->info().in_domain() ) {
			std::vector<Point2D> trianglePoints;

			// Obterner el triángulo del respectivo faces
			Triangle2D triangleFace = _cdt.triangle(fit) ;
			double triangleArea = triangleFace.area();
			//double ratioAreas = triangleArea / this->getArea();
			
			// La cantidad de puntos dentro del triangulo depende de su área y del tamaño
			// de los cuadrantes
			
			uint32_t qSize = _myEnv->getGrid()._quadSize;
			uint32_t tPointsInsideTriangle = triangleArea / (qSize*qSize) + 2;
			
			tPointsInsideTriangle *= 8;
			
			//std::cout << "area triangle: " << triangleArea << " -> pointsInside: " << tPointsInsideTriangle << std::endl;
			

			// Crear un generador de puntos aleatorios dentro del triangulo "triangleFace"
			// Ref: https://doc.cgal.org/latest/Generator/Generator_2random_points_triangle_2_8cpp-example.html
			CGAL::Random_points_in_triangle_2<Point2D> generator( triangleFace );

			// Generar 'tPointsInsideTriangle' puntos y almacenarlos en "trianglePoints"
			CGAL::cpp11::copy_n(generator, tPointsInsideTriangle, std::back_inserter(trianglePoints));

			// Aleatorizar la posición de los puntos dentro del vector "trianglePoints"
			std::random_shuffle(trianglePoints.begin(),trianglePoints.end());

			// Seleccionar el primero y colocarlo en el vector "points"
			//points.push_back(trianglePoints[0]);
			
			// Agregar puntos según el porcentaje de area del triangula
			// con respecto al area total.
			//points.insert(points.end(), trianglePoints.begin(), std::next(trianglePoints.begin(), 100*ratioAreas));
			points.insert(points.end(), trianglePoints.begin(), trianglePoints.end());
			
			
			// Con ese nuevo punto, recalcular el centroide de la zona
			double avgCentroideX = _avgCentroidX(trianglePoints[0][0]);
			double avgCentroideY = _avgCentroidY(trianglePoints[0][1]);
		
			_centroid = Point2D(avgCentroideX, avgCentroideY);
		}
	}
	
	// Procesar los patch agents a la zona
	
	Point2D IDxyMin = Point2D(50000, 50000);
	Point2D IDxyMax = Point2D(-50000, -50000);
	for(const auto& fooPoint : points){
		uint32_t quadId;
		
		quadId = _myEnv->getQuadId(fooPoint);
		PatchAgent* pAgent = _myEnv->getPatchAgent(quadId);
		
		// Agregar id del patch Agent a la zona
		_patchAgentsInZone.insert(quadId);
		
		// Asigna al patchAgent la zona a la que pertenece.
		pAgent->setZone(this);
		
		// Si la zone es inundable,
		// cada patch agent se marca como inundable
		if(_zoneType == "flood"){
			pAgent->isFloodable(true);
			pAgent->setMaxLevelFlood(_maxLevelFlood);
			
		}

		PatchAgent::quad_t quadInfoPAgent = pAgent->getQuadInfo();
		
		uint32_t idX = quadInfoPAgent.idX;
		uint32_t idY = quadInfoPAgent.idY;
		
		Point2D pNew = Point2D(idX,idY);
		
		//Determinar xMin,xMax, yMin, yMax
		double xMin, xMax;
		xMin = std::min(IDxyMin[0], pNew[0]);
		xMax = std::max(IDxyMax[0], pNew[0]);
		
		double yMin, yMax;
		yMin = std::min(IDxyMin[1], pNew[1]);
		yMax = std::max(IDxyMax[1], pNew[1]);
		
		IDxyMin = Point2D(xMin, yMin);
		IDxyMax = Point2D(xMax, yMax);
		
	}
	
	_NSWEAgentsInZone[0] = IDxyMax[1]; //N
	_NSWEAgentsInZone[1] = IDxyMin[1]; //S
	_NSWEAgentsInZone[2] = IDxyMin[0]; //W
	_NSWEAgentsInZone[3] = IDxyMax[0]; //E
}
*/

const std::set<uint32_t>& Zone::patchAgentsInZone(void)
{
	return(_patchAgentsInZone);
}

/*
 * Devuelve los identificadores minimos y maximos de los	
 * patch agents de la zona:
 *
 * _NSWEAgentsInZone[0]: idY más al norte
 * _NSWEAgentsInZone[1]: idY más al sur
 * _NSWEAgentsInZone[2]: idX más al oeste
 * _NSWEAgentsInZone[3]: idX más al este
 */
void Zone::setNSWEPatchAgentsInZone(void)
{
	Point2D IDxyMin = Point2D(50000, 50000);
	Point2D IDxyMax = Point2D(-50000, -50000);
	for(auto& patchId : _patchAgentsInZone){
		PatchAgent* pAgent = _myEnv->getPatchAgent(patchId);
		
		PatchAgent::quad_t quadInfoPAgent = pAgent->getQuadInfo();
		
		uint32_t idX = quadInfoPAgent.idX;
		uint32_t idY = quadInfoPAgent.idY;
		
		Point2D pNew = Point2D(idX,idY);
		
		//Determinar xMin,xMax, yMin, yMax
		double xMin, xMax;
		xMin = std::min(IDxyMin[0], pNew[0]);
		xMax = std::max(IDxyMax[0], pNew[0]);
		
		double yMin, yMax;
		yMin = std::min(IDxyMin[1], pNew[1]);
		yMax = std::max(IDxyMax[1], pNew[1]);
		
		IDxyMin = Point2D(xMin, yMin);
		IDxyMax = Point2D(xMax, yMax);
	}
	
	_NSWEAgentsInZone[0] = IDxyMax[1]; //N
	_NSWEAgentsInZone[1] = IDxyMin[1]; //S
	_NSWEAgentsInZone[2] = IDxyMin[0]; //W
	_NSWEAgentsInZone[3] = IDxyMax[0]; //E
	
	
}

const Zone::NSWEPatchAgentsInZone_t& Zone::getNSWEPatchAgentsInZone(void)
{
	return(_NSWEAgentsInZone);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// private methods
// 

// Mark facets that are inside the domain bounded by the polygon
// Ref: https://doc.cgal.org/latest/Triangulation_2/Triangulation_2_2polygon_triangulation_8cpp-example.html
void Zone::mark_domains(CDT& ct,
                        CDT::Face_handle start,
                        int index,
                        std::list<CDT::Edge>& border )
{

	if(start->info().nesting_level != -1) {
		return;
	}

	std::list<CDT::Face_handle> queue;
	queue.push_back(start);

	while(! queue.empty()) {
		CDT::Face_handle fh = queue.front();
		queue.pop_front();
		if(fh->info().nesting_level == -1) {
			fh->info().nesting_level = index;
			for(int i = 0; i < 3; i++) {
				CDT::Edge e(fh,i);
				CDT::Face_handle n = fh->neighbor(i);
				if(n->info().nesting_level == -1) {
					if(ct.is_constrained(e)) {
						border.push_back(e);
					} else {
						queue.push_back(n);
					}
				}
			}
		}
	}
}
//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void Zone::mark_domains(CDT& cdt)
{
	for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
		it->info().nesting_level = -1;
	}

	std::list<CDT::Edge> border;
	mark_domains(cdt, cdt.infinite_face(), 0, border);

	while(! border.empty()) {
		CDT::Edge e = border.front();
		border.pop_front();
		CDT::Face_handle n = e.first->neighbor(e.second);

		if(n->info().nesting_level == -1) {
			mark_domains(cdt, n, e.first->info().nesting_level+1, border);
		}
	}
}


////////////////////////////////////////////////////////////////////////
// zoneSimilarity implementation
//
void zonesSimilarity::sortZones(Zone& a, Zone& b)
{
	Zone tmp;
	
	Vector2D zoneA_wh = a.getRequiv();
	Vector2D zoneB_wh = b.getRequiv();
	
	//if(a._rh > b._rh){
	if(zoneA_wh.y() > zoneB_wh.y()){
		tmp = a;
		a = b;
		b = tmp;
	}
}


double zonesSimilarity::jaccardIndex(Zone a, Zone b)
{
	const GeographicLib::Geodesic& geod(GeographicLib::Geodesic::WGS84());
	double jaccard = 0.0;
	
	zonesSimilarity::sortZones(a, b);
	
	
	double distancia;
	Point2D centroidA = a.getCentroidWGS84();
	Point2D centroidB = b.getCentroidWGS84();
	
	geod.Inverse(centroidA.y(), centroidA.x(), centroidB.y(), centroidB.x(), distancia);
	
	Vector2D zA_wh = a.getRequiv();
	Vector2D zB_wh = b.getRequiv();
	
	//if(distancia <= abs(b._rw - a._rw)){
	if( distancia <= abs(zB_wh.x() - zA_wh.x()) ){
		
		//if(a._rw <= b._rw){//Caso 1
		if(zA_wh.x() <= zB_wh.x()){//Caso 1
			//jaccard = (a._rw*a._rh)/(b._rw*b._rh);
			jaccard = (zA_wh.x()*zA_wh.y())/(zB_wh.x()*zB_wh.y());
		}
		else{ //Caso 2
			//jaccard = (b._rw*a._rh)/(a._rw*a._rh+b._rw*b._rh-b._rw*a._rh);
			jaccard = (zB_wh.x()*zA_wh.y()) / (zA_wh.x()*zA_wh.y() + zB_wh.x()*zB_wh.y() - zB_wh.x()*zA_wh.y());
		}
	}
	else if(distancia > abs(zB_wh.x() - zA_wh.x())  && distancia <= (zB_wh.x() + zA_wh.x()) ){
		//double n = 2*a._rh*(a._rw+b._rw-distancia);
		double n = 2*zA_wh.y()*(zA_wh.x() + zB_wh.x() - distancia);
		//jaccard = n/(4*(a._rw*a._rh+b._rw*b._rh) - n);
		jaccard = n/(4*(zA_wh.x()*zA_wh.y() + zB_wh.x()*zB_wh.y()) - n);
	}
	else{
		jaccard = 0;
	}

	return(jaccard);
}





