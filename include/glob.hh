#ifndef _GLOB_H_
#define _GLOB_H_
#include <CGAL/Line_2.h>
#include <CGAL/Origin.h>
#include <CGAL/Polygon_2.h>
//#include <CGAL/Cartesian.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>


#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <osrm/match_parameters.hpp>
#include <osrm/nearest_parameters.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/table_parameters.hpp>
#include <osrm/trip_parameters.hpp>
#include <osrm/coordinate.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/json_container.hpp>
#include <osrm/osrm.hpp>
#include <osrm/status.hpp>

#include <exception>
#include <iostream>
#include <utility>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <mutex>
#include <list>
#include <random>

#include <json.hpp>
#include <kdtree++/kdtree.hpp>

#define CGAL_HAS_THREADS

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Aff_transformation_2<K> Transformation;
typedef CGAL::Line_2<K>    Line2D;
typedef CGAL::Point_2<K>   Point2D;
typedef CGAL::Vector_2<K>  Vector2D;
typedef CGAL::Polygon_2<K> Polygon2D;

typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K>                        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>              Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds>        CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT>                  Mesh_2_criteria;

using json=nlohmann::json;
using namespace GeographicLib;

#define CLOSE_ENOUGH          1.0
#define RANDOMWALKWAY_RADIUS  500.0
#define CALIBRATION_TIME      250U
#define ATTRACTION_RADIUS     2.0
#define SAVE                  10

enum model_t {SHORTESTPATH=9366416273040049814U,FOLLOWTHECROWD=10676684734677566718U,RANDOMWALKWAY=5792789823329120861U,WORKINGDAY};

#endif
