#pragma once

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