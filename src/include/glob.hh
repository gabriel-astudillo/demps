#ifndef _GLOB_H_
#define _GLOB_H_

#include <sys/resource.h>

#include <exception>
#include <iostream>
#include <utility>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <array>
#include <string>
#include <mutex>
#include <list>
#include <map> 
#include <unordered_map>
#include <random>
#include <omp.h>
#include <iomanip>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <filesystem>
#include <csignal>
//#include <ranges> only c++20

#include <json.hpp>
using json=nlohmann::json;
#include <progressBar.hpp>
//#include <utils.hh>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>


#include <CGAL/Line_2.h>
#include <CGAL/Origin.h>
#include <CGAL/Polygon_2.h>
//#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>  //new 
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

#define CGAL_HAS_THREADS


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

#include <restclient-cpp/restclient.h>
enum CURLerrorCode {COULDNT_CONNECT = 7};

struct FaceInfo2 {
	FaceInfo2() {}
	int nesting_level;
	bool in_domain()
	{
		return nesting_level%2 == 1;
	}
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel      K;//new
//typedef CGAL::Simple_cartesian<double> K;

typedef CGAL::Aff_transformation_2<K> Transformation;

typedef CGAL::Line_2<K>     Line2D;
typedef CGAL::Point_2<K>    Point2D;
typedef CGAL::Triangle_2<K> Triangle2D;//new
typedef CGAL::Vector_2<K>   Vector2D;
typedef CGAL::Polygon_2<K>  Polygon2D;

typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>    Fbb;//new
//typedef CGAL::Delaunay_mesh_face_base_2<K>                        Fb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>        Fb;//new
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>              Tds;

typedef CGAL::Exact_predicates_tag                                Itag;//new
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds, Itag>  CDT;//new
//typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds>        CDT;

typedef CGAL::Delaunay_mesh_size_criteria_2<CDT>                  Mesh_2_criteria;


using namespace GeographicLib;

extern bool        g_showProgressBar;
extern bool        g_panicModelEnable;
extern bool        g_elevationModelEnable;
extern bool        g_elevationPatchDataValid;
extern bool        g_debrisModelEnable;
extern bool        g_floodModelEnable;
extern bool        g_agentsOut;
extern float       g_closeEnough;
extern float       g_randomWalkwayRadius;
extern float       g_attractionRadius;
extern uint32_t    g_currTimeSim;
extern std::string g_baseDir;
extern uint32_t    g_AgentsMem;
extern float       g_deltaT;
extern uint32_t    g_totalAgentsInSim;

//Variables globales para medir tiempo
extern uint32_t g_timeExecMakeAgents;
extern uint32_t g_timeExecCal;
extern uint32_t g_timeExecSim;

extern std::vector<std::string> g_logZonesDensity;
extern std::vector<uint32_t>    g_logUsePhone;
extern std::vector<std::string> g_logVelocity;
extern std::vector<std::string> g_logSIRpanic;
extern std::vector<std::string> g_logDeceasedAgents;
extern std::ostringstream       g_logStepDelay;

//enum model_t {ShortestPath=0, FollowTheCrowd=1, RandomWalkway=2, WorkingDay, SNITCH=666};
enum model_t {Residents=0, Visitors_I=1, Visitors_II=2};


extern std::map<std::string, model_t> model_map;

/*
//               Ts                   Ak                   N(k,s)    %N     min(k) max(k) meanDist
extern std::map< uint32_t,  std::map< uint32_t, std::tuple<uint32_t, float, float, float, float> > > g_sdataDistSafeZoneByAgeGroup;
//               Ts                   Zn                      E(n,s)   %E(n,s) tevac_min  tevac_max  tevac_mean
extern std::map< uint32_t,  std::map< std::string, std::tuple<uint32_t, float, float    , float    , float      > > > g_sdataEvacTimeByZone;
*/

////////////////////////////////////////////////////////////////////////////////////////////////
// versión con los índices en std::string para permitir la conversión a JSON en forma automática
//
//                         Ts                      Ak                      N(k,s)    %N     min(k) max(k) meanDist
extern std::map< std::string,  std::map< std::string, std::tuple<uint32_t, float, float, float, float> > > g_sdataDistSafeZoneByAgeGroup;
//                         Ts                      Zn                      E(n,s)   %E(n,s) tevac_min  tevac_max  tevac_mean
extern std::map< std::string,  std::map< std::string, std::tuple<uint32_t, float, float    , float    , float      > > > g_sdataEvacTimeByZone;

extern std::vector<double> g_evacTime;

#endif
