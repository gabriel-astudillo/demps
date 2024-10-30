#pragma once

/*
	
	================ Ejemplos de uso =================
	dtw comp01;
	comp01.referenceSerie(reference);
	comp01.querySerie(query);
	comp01.windowSize(0);
	comp01.run();

	//Comparar otra serie con la serie de referencia.
	comp01.querySerie(query02);
	comp01.run();
	

	dtw comp01 = createDTW()
		.referenceSerie(ref)
	    .querySerie(qry)
		.windowSize(0);
	
	comp01.run();
	
*/

#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <map>

#include <utils.hh>

/*
namespace utils{
	//public:
	typedef std::map<uint32_t, double> timeSerie_t ;
	typedef std::vector<double>        dataSerie_t;
	typedef std::vector<uint32_t>      timestampSerie_t;
	
	namespace key{
		const std::string evac = "evacByZone";
		const std::string nonEvac = "nonEvacByAge";
	}
	
}
*/

class createDTW;


class dtw {
public:
	typedef std::vector< std::tuple<int32_t, int32_t> > warpPath_t;
	typedef std::map<uint32_t, double> timeSerie_t ;
	typedef std::vector<double>        dataSerie_t;
	typedef std::vector<uint32_t>      timestampSerie_t;
	
private:
	dataSerie_t _referenceSerie;
	dataSerie_t _querySerie;
	
	uint32_t    _windowSize;
	double*     _dtwMatrix;
	
	uint32_t    _lenA, _lenB;
	
	double      _DTWdistance;
	double      _DTWdistanceNorm;
	double      _DTWdistanceOE;
	double      _DTWavgTimeShift;
	double      _DTWavgTimeShiftOE;
	warpPath_t  _DTWwarpPath;
	warpPath_t  _DTWwarpPathOE;
	
	double distance(const double& A, const double& B);
	double warpPathFrom(const int32_t& i, const int32_t& j, warpPath_t& timeShifts);
	
	void setMatrixInitCond();
	void calcDTWdistance();
	
public:
	dtw();
	dtw(const createDTW& params); 
	~dtw();
	
	void referenceSerie(const dataSerie_t& referenceSerie);
	void querySerie(const dataSerie_t& querySerie);
	
	uint32_t windowSize();
	void windowSize(const uint32_t& ws);
	double DTWdistance();
	double DTWdistanceNorm();
	double DTWdistanceOE();
	double DTWavgTimeShift();
	double DTWavgTimeShift(const timestampSerie_t& timestampQuery, const timestampSerie_t& timestampReference);
	double DTWavgTimeShiftOE();
	double DTWavgTimeShiftOE(const timestampSerie_t& timestampQuery, const timestampSerie_t& timestampReference);
	warpPath_t DTWwarpPath();
	warpPath_t DTWwarpPathOE();
	
	void run();
	
};

class createDTW{
private:
	uint32_t _windowSize;
	//std::vector<double> _referenceSerie;
	//std::vector<double> _querySerie;
	dtw::dataSerie_t _referenceSerie;
	dtw::dataSerie_t _querySerie;
	
	
public:
	createDTW() = default;
	//createDTW(const createDTW& b) = default;
	
	
	createDTW& referenceSerie(const dtw::dataSerie_t& referenceSerie){
		_referenceSerie = referenceSerie;
		return *this;
	}
	
	createDTW& querySerie(const dtw::dataSerie_t& querySerie){
		_querySerie = querySerie;
		return *this;
	}
	
	createDTW& windowSize(const uint32_t& ws){
		_windowSize = ws;
		return *this;
	}
	
	friend class dtw;
};


