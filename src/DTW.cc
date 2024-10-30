#include <DTW.hh>


dtw::dtw()
{
	_windowSize = 0;
}

dtw::dtw(const createDTW& params): 
	_referenceSerie(params._referenceSerie),
	_querySerie(params._querySerie),
	_windowSize(params._windowSize)
{
	_lenB = _referenceSerie.size();
	_lenA = _querySerie.size();
}

dtw::~dtw()
{

}

void dtw::referenceSerie(const dtw::dataSerie_t& referenceSerie)
{
	_referenceSerie = referenceSerie;
	_lenB = _referenceSerie.size();	
}

void dtw::querySerie(const dtw::dataSerie_t& querySerie)
{
	_querySerie = querySerie;
	_lenA = _querySerie.size();
}

uint32_t dtw::windowSize()
{
	return(_windowSize);
}

void dtw::windowSize(const uint32_t& ws)
{
	_windowSize = ws;
}

double dtw::DTWdistance()
{
	return(_DTWdistance);
}

double dtw::DTWdistanceNorm()
{
	return(_DTWdistanceNorm);
}

double dtw::DTWdistanceOE()
{
	return(_DTWdistanceOE);
}

double dtw::DTWavgTimeShift()
{
	_DTWavgTimeShift = 0;
	for(const auto& [i, j] : _DTWwarpPath){
		_DTWavgTimeShift += i-j;
	}
	_DTWavgTimeShift /= _DTWwarpPath.size();
	
	return(_DTWavgTimeShift);
}

double dtw::DTWavgTimeShift(const timestampSerie_t& timestampQuery, const timestampSerie_t& timestampReference)
{
	_DTWavgTimeShift = 0.0;
	uint32_t total = 0;
	for(const auto& [i,j] : _DTWwarpPath){
		if( _querySerie[i]>0 ){
			int32_t deltaT = timestampQuery[i]-timestampReference[j];
			_DTWavgTimeShift += deltaT;
			total++;
		}
	}
	_DTWavgTimeShift /= total;
	
	return(_DTWavgTimeShift);
}

double dtw::DTWavgTimeShiftOE()
{
	_DTWavgTimeShiftOE = 0;
	for(const auto& [i,j] : _DTWwarpPathOE){
		_DTWavgTimeShiftOE += i-j;
	}
	_DTWavgTimeShiftOE /= _DTWwarpPathOE.size();
	
	return(_DTWavgTimeShiftOE);
}

double dtw::DTWavgTimeShiftOE(const timestampSerie_t& timestampQuery, const timestampSerie_t& timestampReference)
{	
	_DTWavgTimeShiftOE = 0.0;
	uint32_t total = 0;
	for(const auto& [i,j] : _DTWwarpPathOE){
		if( _querySerie[i]>0 ){
			int32_t deltaT = timestampQuery[i]-timestampReference[j];
			_DTWavgTimeShiftOE += deltaT;
			total++;
		}
	}
	_DTWavgTimeShiftOE /= total;
	
	return(_DTWavgTimeShiftOE);
}

dtw::warpPath_t dtw::DTWwarpPath()
{
	return(_DTWwarpPath);
}
dtw::warpPath_t dtw::DTWwarpPathOE()
{
	return(_DTWwarpPathOE);
}

void dtw::run()
{
	if(_windowSize == 0){
		//_windowSize = _lenB;
		_windowSize = std::max(_lenA,_lenB);
	}
	// matriz de tamaño lenA*lenB
	uint32_t matrixSize = (_lenA)*(_lenB);
	//std::cout << "|Query|=" << _lenA << ", |Ref|=" << _lenB << std::endl;
	
	_dtwMatrix = new double[matrixSize];
	
	this->setMatrixInitCond();
	
	// Determina:
	//  * _DTWdistance;
	//  * _DTWdistanceOE;
	this->calcDTWdistance();
	
	
	delete[] _dtwMatrix;
}

void dtw::setMatrixInitCond()
{
	// Se llena la matriz con inf 
	for(size_t i = 0; i < _lenA; i++){
		for(size_t j = 0; j < _lenB; j++){
			_dtwMatrix[ i*(_lenB) +  j  ] = std::numeric_limits<double>::infinity();
		}
	}
	
	// Condiciones de borde de la matriz, considerando el tamaño de la ventana
	_dtwMatrix[0] = dtw::distance(_referenceSerie[0], _querySerie[0]);
	for(size_t i = 1; i <= std::min(_lenA-1, _windowSize); i++){
		_dtwMatrix[i*_lenB + 0] = dtw::distance(_querySerie[i], _referenceSerie[0] ) + _dtwMatrix[(i-1)*_lenB + 0];
	}
	
	for(size_t j = 1; j <= std::min(_lenB-1, _windowSize); j++){
		_dtwMatrix[0*_lenB + j] = dtw::distance(_querySerie[0], _referenceSerie[j] ) + _dtwMatrix[0*_lenB + (j-1)];
	}
	
}

void dtw::calcDTWdistance()
{
	double distanceMax;
	
	distanceMax = 0;
	// Determinar la distancia de las series en base al Warp Path óptimo
	for(size_t i = 1; i < _lenA; i++){
		int jMin, jMax;
		
		jMin = std::max(1, int(i - _windowSize) );//1;
		jMax = std::min((int)(_lenB-1), int(i + _windowSize));//lenB;
		
		for(int j = jMin; j <= jMax; j++){
			
			double cost;
			cost = dtw::distance(_querySerie[i], _referenceSerie[j]);
			
			std::vector<double> nums;
			double lastMin;
			
			nums.push_back(   cost + _dtwMatrix[i    *(_lenB) +  j-1] );
			nums.push_back( 2*cost + _dtwMatrix[(i-1)*(_lenB) +  j-1] );
			nums.push_back(   cost + _dtwMatrix[(i-1)*(_lenB) +  j  ] );
			
			
			auto result = std::min_element(nums.begin(), nums.end());
			
			lastMin = *result;
			
			_dtwMatrix[ i*(_lenB) +  j ] = lastMin;
			
			if( lastMin > distanceMax){
				distanceMax = lastMin;
			}
			
		}
	}
	
	_DTWdistance = _dtwMatrix[(_lenA-1)*(_lenB) +  (_lenB-1)  ];
	_DTWdistanceNorm = _DTWdistance / (_lenA + _lenB);
	
	
	_DTWwarpPath.clear();
	warpPathFrom(_lenA-1, _lenB-1, _DTWwarpPath);
	
	
	//////////////////////
	// Etapa Open End DTW
	//
	
	double costMin = std::numeric_limits<double>::infinity();
	uint32_t colMin = 0;
	for(size_t j = 0; j < _lenB; j++){
		double cost = _dtwMatrix[ (_lenA-1) * _lenB +  j ] ;
		if(cost <= costMin){
			costMin = cost;
			colMin = j;
		}
	}
	
	_DTWdistanceOE     = costMin;
	
	_DTWwarpPathOE.clear();
	warpPathFrom(_lenA-1, colMin, _DTWwarpPathOE);
	
}


// Recorre la WP desde el final hasta el inicio
// buscando las celdas con menor costo.
// Almacena los puntos de la WP en warpPath
double dtw::warpPathFrom(const int32_t& i, const int32_t& j, warpPath_t& warpPath)
{
	//std::cout << "count: " << timeShifts.size() << std::endl;
	//std::cout << "cell: " << i << "," << j << std::endl;
	//std::cout <<  dtwMatrix[i*(lenB) +  j] << std::endl;
	warpPath.push_back( std::make_tuple(i,j) );
	
	if(i == 0 && j == 0){
		return(0);
	}
	
	std::vector<double> nums;
	
	if(j-1 >= 0){
		nums.push_back( _dtwMatrix[i    *(_lenB) +  j-1] );
	}
	else{
		nums.push_back( std::numeric_limits<double>::infinity() );
	}
	
	if((i-1 >= 0) && (j-1 >= 0)){
		nums.push_back( _dtwMatrix[(i-1)*(_lenB) +  j-1] );
	}
	else{
		nums.push_back( std::numeric_limits<double>::infinity() );
	}	

	
	if(i-1 >= 0 ){
		nums.push_back( _dtwMatrix[(i-1)*(_lenB) +  j  ] );
	}
	else{
		nums.push_back( std::numeric_limits<double>::infinity() );
	}
	
	auto result = std::min_element(nums.begin(), nums.end());
	int pos = std::distance(nums.begin(), result);
	
	
	switch(pos){
		case 0 : 
			warpPathFrom(i, j-1, warpPath );
			break;
		case 1 : 
			warpPathFrom(i-1, j-1, warpPath); 
			break;
		case 2 : 
			warpPathFrom(i-1, j, warpPath); 
			break;
	}
	
	return(0);
}


double dtw::distance(const double& A, const double& B)
{
	return( std::abs(A - B) );
}

