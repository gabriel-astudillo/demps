#include <zone.hh>

Zone::Zone(void) {
    ;
}
Zone::Zone(const json &_freference_point,const json &_fpolygon) {
	this->_projector=LocalCartesian(_freference_point["features"][0]["geometry"]["coordinates"][1],_freference_point["features"][0]["geometry"]["coordinates"][0],0,Geocentric::WGS84());

	if(_fpolygon["geometry"]["type"]!="Polygon") {
		std::cerr << "Error::input feature is not a polygon" << std::endl;
		exit(EXIT_FAILURE);
	}
	
	this->_nameID = _fpolygon["properties"]["nameID"].get<std::string>();	
	
	for(auto& fpoint : _fpolygon["geometry"]["coordinates"][0]) {
		double x,y,z,h;
		
		h = 0.0;
		this->_projector.Forward(fpoint[1],fpoint[0],h,x,y,z);
		this->_polygon.push_back(Point2D(x,y));
	}
	
	_area          = this->_polygon.area();
	_agentsDensity = 0.0;

	// Insert the polygons into a constrained triangulation
	this->_cdt.insert_constraint(this->_polygon.vertices_begin(),this->_polygon.vertices_end(),true);

	// Mark facets that are inside the domain bounded by the polygon
	// Ref: https://doc.cgal.org/latest/Triangulation_2/Triangulation_2_2polygon_triangulation_8cpp-example.html
	mark_domains(_cdt);
	
}

Zone::Zone(const Zone &_z) {
	this->_cdt     = _z._cdt;
	this->_nameID  = _z._nameID;
	this->_polygon = _z._polygon;
	this->_area    = _z._area;
	this->_projector     = _z._projector;
	this->_agentsDensity = _z._agentsDensity;
	this->_agentsInZone  = _z._agentsInZone;
}

Zone::~Zone(void) {
    ;
}

Zone& Zone::operator=(const Zone &_z) {
	this->_cdt     = _z._cdt;
	this->_nameID  = _z._nameID;
	this->_polygon = _z._polygon;
	this->_area    = _z._area;
	this->_projector     = _z._projector;
	this->_agentsDensity = _z._agentsDensity;
	this->_agentsInZone  = _z._agentsInZone;
	return(*this);
}

bool Zone::pointIsInside(const Point2D& testPoint){
	
	CGAL::Bounded_side bside = CGAL::bounded_side_2(this->_polygon.vertices_begin(), this->_polygon.vertices_end(), testPoint, K() );
	if (bside == CGAL::ON_BOUNDED_SIDE) { 
	    return(true);
	} 
	else{
		return(false);
	}
}

void Zone::addAgent(const uint32_t& idAgent){
	//#pragma omp critical
	{
	_agentsInZone.insert(idAgent);
	}
}

void Zone::deleteAgent(const uint32_t& idAgent){
	//#pragma omp critical
	{
	_agentsInZone.erase(idAgent);
	}
}

void Zone::updateAgentsDensity(void){
	//#pragma omp critical
	{
		_agentsDensity = _agentsInZone.size() / _area;
	}
	
}

uint32_t Zone::getTotalAgents(void){
	return(_agentsInZone.size());
}

double Zone::getAgentsDensity(void){
	return(_agentsDensity);
}

std::string Zone::getNameID(void){
	return(_nameID);
}

Point2D Zone::generate(void) {
	std::vector<Point2D> points;

	/*
		El CDT está constituido por "faces", que son triangulos que están delimitados
		por el contorno del polígono "_polygon".
		
		Por cada "faces", se toma el respectivo triangulo y se crean puntos aleatorios
		dentro de él.
		
		Luego, se escoge uno de ellos y se agrega a la coleccion de posibles puntos 
		del agente.
	
		Al finilizar, se aleatorizan los puntos y se selecciona uno de ellos.
	*/
	for (CDT::Finite_faces_iterator fit=_cdt.finite_faces_begin(); fit!=_cdt.finite_faces_end(); ++fit) {
		if ( fit->info().in_domain() ){
			std::vector<Point2D> trianglePoints;
			
			// Obterner el triángulo del respectivo faces 
			Triangle2D triangleFace = _cdt.triangle(fit) ;
			
			// Crear un generador de puntos aleatorios dentro del triangulo "triangleFace"
			// Ref: https://doc.cgal.org/latest/Generator/Generator_2random_points_triangle_2_8cpp-example.html
			CGAL::Random_points_in_triangle_2<Point2D> generator( triangleFace );
			
			// Generar 100 puntos y almacenarlos en "trianglePoints"
			CGAL::cpp11::copy_n(generator,100,std::back_inserter(trianglePoints));
			
			// Aleatorizar la posición de los puntos dentro del vector "trianglePoints"
			std::random_shuffle(trianglePoints.begin(),trianglePoints.end());
			
			// Seleccionar el primero y colocarlo en el vector "points"
			points.push_back(trianglePoints[0]);
		}
	}
	
	// Aleatorizar la posición de los puntos dentro del vector "points"
	std::random_shuffle(points.begin(),points.end());
	
	// Seleccionar el primero
	return(points[0]);
}


// Mark facets that are inside the domain bounded by the polygon
// Ref: https://doc.cgal.org/latest/Triangulation_2/Triangulation_2_2polygon_triangulation_8cpp-example.html
void Zone::mark_domains(CDT& ct, 
             CDT::Face_handle start, 
             int index, 
             std::list<CDT::Edge>& border ) {
				 	
	if(start->info().nesting_level != -1){
		return;
	}
	
	std::list<CDT::Face_handle> queue;
	queue.push_back(start);
	
	while(! queue.empty()){
		CDT::Face_handle fh = queue.front();
		queue.pop_front();
		if(fh->info().nesting_level == -1){
			fh->info().nesting_level = index;
			for(int i = 0; i < 3; i++){
				CDT::Edge e(fh,i);
				CDT::Face_handle n = fh->neighbor(i);
				if(n->info().nesting_level == -1){
					if(ct.is_constrained(e)){
						border.push_back(e);
					}
					else{ 
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
void Zone::mark_domains(CDT& cdt) {
	for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
		it->info().nesting_level = -1;
	}
	
	std::list<CDT::Edge> border;
	mark_domains(cdt, cdt.infinite_face(), 0, border);
	
	while(! border.empty()){
		CDT::Edge e = border.front();
		border.pop_front();
		CDT::Face_handle n = e.first->neighbor(e.second);
		
		if(n->info().nesting_level == -1){
			mark_domains(cdt, n, e.first->info().nesting_level+1, border);
		}
	}
}

