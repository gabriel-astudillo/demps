#pragma once

#include <json.hpp>
using json=nlohmann::json;

#include <vector>
#include "restclient-cpp/connection.h"
#include <restclient-cpp/restclient.h>
#include <uuid/uuid.h>

#include <DTW.hh>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h> 
typedef CGAL::Exact_predicates_inexact_constructions_kernel   K;
typedef CGAL::Point_2<K>    Point2D;
typedef CGAL::Vector_2<K>   Vector2D;


#define _USE_MATH_DEFINES
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>
using namespace GeographicLib;

#include <PGM.hpp>
#include <Timer.hpp>

namespace utils{
	
	struct utilsException : public std::exception {
		std::string msg;
		const char * what () const throw () {
			return(msg.c_str()) ;
		}
	};
	
	typedef std::map<uint32_t, double> timeSerie_t ;
	typedef std::map<std::string, double> timeSerie_json_t ;
	typedef std::vector<double>        dataSerie_t;
	typedef std::vector<uint32_t>      timestampSerie_t;
	typedef std::string                uuidSimulation_t;
	typedef double                     dtwDistance_t;
	typedef double                     timeMeasure_t;
	typedef double                     radius_t;
	typedef double                     distance_t;
	typedef std::string                simDescription_t;
	typedef std::string                zoneNameID_t;
	
	typedef double                     heatMapDistance_t;
	typedef std::string                heatMapFilePath_t;
	typedef PGM                        heatMapData_t;

	
	//typedef std::map<utils::uuidSimulation_t, std::tuple<std::string, utils::timeSerie_t> >                          listTS_t;
	typedef std::map<utils::uuidSimulation_t, std::tuple<utils::simDescription_t, utils::timeSerie_t, json> >                    listTS_t;
	
	//typedef std::vector< std::tuple<utils::uuidSimulation_t, utils::timeSerie_t, utils::radius_t, utils::listTS_t> > indexTS_t;
	typedef std::vector< std::tuple<utils::uuidSimulation_t, utils::simDescription_t, utils::timeSerie_t, json, utils::radius_t, utils::listTS_t> > indexTS_t;
	
	//typedef std::map<utils::distance_t, std::tuple<utils::uuidSimulation_t,std::string, utils::timeSerie_t>>         searchResult_t;
	typedef std::map<utils::distance_t, std::tuple<utils::uuidSimulation_t,utils::simDescription_t, utils::timeSerie_t, json >>         searchResult_t;
	
	// tipos de datos para la lista de cluster de heatMaps
	typedef std::map<                 \
		utils::heatMapFilePath_t,     \
		utils::heatMapData_t          \
		>  listHeatMapFiles_t;
	
	typedef std::multimap<            \
		utils::uuidSimulation_t,      \
		std::tuple<                   \
			utils::simDescription_t,  \
			utils::heatMapFilePath_t  \
			/*utils::heatMapData_t*      \*/
			>                         \
		> listHMap_t;
	
	typedef std::vector<\
		std::tuple<                   \
			utils::uuidSimulation_t,  \
			utils::simDescription_t,  \
			utils::heatMapFilePath_t, \
			/*utils::heatMapData_t*,     \*/
			utils::radius_t,          \
			utils::listHMap_t         \
			>                         \
		> indexHMap_t;
	typedef json indexHMapJSON_t;
	
	typedef std::map<                 \
		utils::distance_t,            \
		std::tuple<                   \
			utils::uuidSimulation_t,  \
			utils::simDescription_t,  \
			utils::heatMapFilePath_t  \
			>                         \
		> searchResultHMap_t;
	
	enum CURLerrorCode {COULDNT_CONNECT = 7};
	
	namespace key{
		const std::string evacAll = "evacAll";
		const std::string nonEvacAll = "nonEvacAll";
		const std::string evacByZone = "evacByZone";
		const std::string nonEvac = "nonEvacByAge";
		const std::string zonesInfo = "zonesInfo";
	}
	
	///////////////////////////////////////////////////
	// Estructura para almacenar los
	// datos muestreados de las simulaciones anteriores
	// junto con las métricas de comparación con la
	// simulación en curso
	struct simulationsData{
		utils::uuidSimulation_t uuidSim;
		double                  samplingLevel;
		uint32_t                samplingTime;
		utils::timeSerie_t      ts;
		utils::dtwDistance_t    distance;
		utils::timeMeasure_t    avgTimeShift;
		
		simulationsData() {}
		simulationsData(utils::uuidSimulation_t id, double sLevel, uint32_t sTime, utils::timeSerie_t ts, double dist, double timeShift): 
			uuidSim(id), samplingLevel(sLevel), samplingTime(sTime), ts(ts), distance(dist), avgTimeShift(timeShift)	{}
	};
	
	///////////////////////////////////////////////////////////
	// Las estructuras con los datos de simulaciones anteriores
	// se almacenan en un map, indexado por la distancia DTW.
	// Esto permite que las simulaciones más cercanas sean las 
	// primeras en el map.
	// Los datos de las simulaciones anteriores se almacenan en 
	// la estructura 'utils::simulationsData'
	typedef std::map<utils::dtwDistance_t, utils::simulationsData> simsComparisonMap_t;
	
	
	utils::uuidSimulation_t get_uuid();
	
	void elevationDataToVector(std::string fileName, std::map<int32_t, std::tuple<double, double,int32_t> >& elevData);
	
	void restClient_get(const std::string restURL, json& response);
	
	//void transformData(json response, std::string KEY, std::map<utils::uuidSimulation_t, std::tuple<std::string, utils::timeSerie_t> >& outTS);
	void transformData(json response, std::string KEY, utils::listTS_t& outTS);
	
	//void transformRawDataJsonTOtsGlobal(json dataRaw, std::string KEY, utils::timeSerie_t& outTS);
	void transformRawDataJsonTOtsGlobal(json dataRaw, std::string KEY, json& outJSON);
	void transformRawDataJsonTOtsLocalByZone(json dataRaw, json& outJSON);	
	
	void extractTimeValueFromTS(const utils::timeSerie_t& ts, utils::timestampSerie_t& timeStamp, utils::dataSerie_t& value, const uint32_t cutIn = 0);
	
	void cutTimeSerie(const utils::timeSerie_t& tsIn, const double& cutValue, utils::timeSerie_t& tsOut);
	
	//void fetchTSdata(std::string queryURL, std::vector<utils::timeSerie_t>& outTS);
	//void fetchTSdata(std::string queryURL, std::map<utils::uuidSimulation_t, std::tuple<std::string, utils::timeSerie_t> >& outTS, bool clearOut = true);
	
	void fetchSimData(const std::string& queryURL, const utils::uuidSimulation_t& uuidSim, json& out);
	
	void timeSerieToJSON(const utils::timeSerie_t& ts, json& timeSerieJSON);
	void JSONToTimeSerie(const json& timeSerieJSON, utils::timeSerie_t& ts);
	utils::timeSerie_t JSONToTimeSerie(const json& timeSerieJSON);
	
	std::tuple<utils::dtwDistance_t, utils::timeMeasure_t> metricsDTW(const utils::timeSerie_t& timeSerie_Qry, const utils::timeSerie_t& timeSerie_Ref);
	double determineMetric(utils::dtwDistance_t d, utils::timeMeasure_t t);
	
	utils::heatMapDistance_t heatMapDistance(heatMapFilePath_t hmFileA, heatMapFilePath_t hmFileB);
	
	void showProgress(const std::string& label, double progress);
	
}

namespace utils{
	class ZoneBasic {
	public:
		//static std::shared_ptr<Environment> _myEnv;
	
	private:
		json    _zoneGeoData;
		json    _zoneEvacTimeSerieJSON;
		Point2D _centroid;      // (x,y)
		Point2D _centroidWSG84; // longitud, latitud
		Vector2D _rEquiv;       //semilados del area rectangular equivalente (rw,rh)
		
		utils::timeSerie_t _zoneEvacTimeSerie;
		
	
		LocalCartesian  _projector;
	
	public:
		ZoneBasic();
		ZoneBasic(json zoneGeoData, json zoneEvacTimeSerieJSON);
		~ZoneBasic(){}
	
		void setCentroidWGS84(double lon, double lat);
		Point2D getCentroidWGS84();
	
		void setRequiv(double rw, double rh);
		Vector2D getRequiv();
		
		utils::timeSerie_t getTimeSerie();
	};
	
	/////////////////////////////////////////////////////////////////////////
	// zoneSimilarity declaration
	//
	struct zonesSimilarity{
	private:
		static void sortZones(ZoneBasic&, ZoneBasic &);
	public:
		static double jaccardIndex(ZoneBasic , ZoneBasic);
	};
	
	
}


////////////////////////////////////////////////////////////////////////////////////////////////
//  Series de tiempo para comparar la simulación en curso con simulaciones
//  anteriores
extern utils::timeSerie_t g_TSevacAll;

