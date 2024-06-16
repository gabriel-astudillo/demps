#pragma once

/////////////////////////////////////////////////////////////////////////////////
//
//   Uso:
//   		PGM varName(fileName, sep);
//
//   Description:
//                Carga los datos de una matriz 2d con tipo de dato T
//                desde el archivo 'fileName' a memoria principal (variable 'varName').
//                
//
//
//   Atributos públicos:
//               Hereda todos los métodos publicos de Matrix<uint32_t>
//               toFile(std::string fileName)
//               invert()
//               pixelate(uint32_t sizeWindow)
//               hEdges()
//
//  Última revisión: 2022-06-07
//

//PGM imgTest(10,10);
//imgTest.maxGreyValue(65535);

//imgTest.value(0,0,30000);
//imgTest.value(9,9,0);
//imgTest.invert();

//imgTest.toFile("imgTest.pgm");

//
/////////////////////////////////////////////////////////////////////////////////

#include <sstream>
#include <iostream>
#include <fstream>
#include <exception>
#include <cmath>
#include <cstdlib>

//#include <Matrix.hpp>
struct matrixException : public std::exception {
	std::string msg;
	const char * what () const throw () {
		return(msg.c_str()) ;
	}
};


template <typename T>
class Matrix{

protected:
	std::ifstream _matrixFile;
	
	std::string _fileName;
	std::string _separador;
	
	
	T *_matrixInMemory;
	uint32_t _nrow, _ncol;
	
	
public:
	Matrix() {}
	Matrix(uint32_t nfil, uint32_t ncol);
	Matrix(std::string fileName, std::string separador);
	Matrix(const Matrix& m);
	~Matrix();
	
	void load();
	 T operator()(uint32_t id_i, uint32_t id_j) const;
	 
	 T value(uint32_t id_i, uint32_t id_j) const;
	void value(uint32_t id_i, uint32_t id_j, T val) const;
	
	uint32_t rows() const {return _nrow;}
	uint32_t cols() const {return _ncol;}
	
	
protected:
	void openFile();
	void loadInMemory();

};

template <typename T>
Matrix<T>::Matrix(uint32_t nfil, uint32_t ncol): _nrow(nfil), _ncol(ncol)
{
	_matrixInMemory = new T[ _nrow * _ncol ];
}

template <typename T>
Matrix<T>::Matrix(std::string fileName, std::string separador): _fileName(fileName), _separador(separador)
{

}

template <typename T>
Matrix<T>::Matrix(const Matrix<T>& m)
{	
	_nrow = m._nrow;
	_ncol = m._ncol;
	
	_matrixInMemory = new T[ _nrow * _ncol ];

	for(size_t i = 0; i < _nrow; i++){	
		for(size_t j = 0; j < _ncol; j++){
			_matrixInMemory[i*_ncol + j] = m._matrixInMemory[i*_ncol + j];
			
		}
	}
}

template <typename T>
Matrix<T>::~Matrix()
{
	delete[] _matrixInMemory;
}

template <typename T>
void Matrix<T>::load()
{
	this->openFile();	
	this->loadInMemory();
}


template <typename T>
void Matrix<T>::openFile()
{
	_matrixFile.open(_fileName);
	if(!_matrixFile.good()){
		matrixException mException;
		mException.msg= "Error en acceso a archivo: " + _fileName;
		throw mException;
	}
}

template <typename T>
void Matrix<T>::loadInMemory()
{
	std::string line;
	size_t index = 0;
	
	std::getline(_matrixFile, line);

	index = line.find(_separador);
	
	_nrow    = std::stoi( line.substr(0, index) );
	_ncol = std::stoi( line.substr(index + _separador.length(), line.length()) );
	
	_matrixInMemory = new T[ _nrow * _ncol ];
	

	for(size_t i = 0; i < _nrow; i++){
		std::getline(_matrixFile, line);
			
		for(size_t j = 0; j < _ncol; j++){
			index = line.find(_separador);
			std::stringstream ssLine( line.substr(0, index)  );
			T value;
			ssLine >> value;

			_matrixInMemory[i*_ncol + j] = value;
			
			line.erase(0, index + _separador.length());	
		}
	}
	_matrixFile.close();
}

template <typename T>
T Matrix<T>::operator()(uint32_t id_i, uint32_t id_j) const
{
	return(_matrixInMemory[id_i*_ncol + id_j]);
}

template <typename T>
T Matrix<T>::value(uint32_t id_i, uint32_t id_j) const
{
	return(_matrixInMemory[id_i*_ncol + id_j]);
}

template <typename T>
void Matrix<T>::value(uint32_t id_i, uint32_t id_j, T val) const
{
	uint32_t idx = id_i*_ncol + id_j;
	_matrixInMemory[idx] = val;
}




class PGM : public Matrix<uint32_t>{

private:
	std::ofstream _pgmFile;
	
	uint32_t _maxGreyValue;
	
	
public:
	PGM() {}
	PGM(uint32_t nfil, uint32_t ncol) : Matrix<uint32_t>(nfil, ncol) {}
	PGM(uint32_t nfil, uint32_t ncol, uint32_t maxGreyValue) : Matrix<uint32_t>(nfil, ncol),  _maxGreyValue(maxGreyValue) {}
	PGM(std::string fileName, std::string separador) : Matrix<uint32_t>(fileName, separador) {}
	PGM(const PGM& p) : Matrix<uint32_t>(p) { _maxGreyValue = p._maxGreyValue; }
	
	
	PGM& operator=( PGM& o){
		
		_nrow = o.rows();
		_ncol = o.cols();
		_maxGreyValue = o._maxGreyValue; 
		_matrixInMemory = new uint32_t[ o.rows() * o.cols() ];
		
		for (uint32_t i = 0; i < o.rows(); i++) {
			for (uint32_t j = 0; j  < o.cols(); j++) {
				//o.value(i, j, this->value(i,j));
				this->value(i, j, o.value(i,j));
				//uint32_t idx = i*_ncol + j;
				//_matrixInMemory[idx] = o.value(i,j);
			}
		}
		return *this;
	}
	
	~PGM() { }
	
	void load(){
		openFile();
		loadInMemory();
	}
	
	void load(std::string fileName, std::string separador){
		_fileName   = fileName;
		_separador  = separador;
		
		this->load();
	}
	
	uint32_t maxGreyValue() const {return _maxGreyValue;}
	void     maxGreyValue(uint32_t maxGreyValue )  { _maxGreyValue = maxGreyValue;}
	
	void toFile(std::string fileName){
		_pgmFile.open(fileName);
		if(!_pgmFile.good()){
			matrixException mException;
			mException.msg= "Error en acceso a archivo: " + fileName;
			throw mException;
		}
	
	
		_pgmFile << "P2\n";
		_pgmFile << this->cols() << " " << this->rows() << "\n";
		_pgmFile << _maxGreyValue << "\n";
	
		for (uint32_t i = 0; i < this->rows(); i++) {
			for (uint32_t j = 0; j <this->cols(); j++) {
				uint32_t grayLevel = this->value(i,j);
				/*
				if(_maxGreyValue > 255){
					// Si el maximum gray value es mayor que 255, entonces hay que representar el
					// nivel de gris en dos números de 8 bits cada uno.
					uint32_t Hvalue = grayLevel >> 8;
					uint32_t Lvalue = grayLevel & 0xFF;
					_pgmFile << Hvalue << " " << Lvalue << " ";
					
				}else{
					_pgmFile << grayLevel<< " ";
				}
				*/
				_pgmFile << grayLevel<< " ";
			}
			_pgmFile << "\n";
		}
		_pgmFile.close();
	}
	
private:
	void loadInMemory(){
		std::string line;
		size_t index = 0;

		std::getline(_matrixFile, line);
		if(line != "P2"){
			matrixException mException;
			mException.msg= "El archivo: " + _fileName + " no es PGM P2";
			throw mException;
		}
	
		std::getline(_matrixFile, line);
		index = line.find(_separador);
		_ncol = std::stoi( line.substr(0, index) );
		_nrow = std::stoi( line.substr(index + _separador.length(), line.length()) );
	
		std::getline(_matrixFile, line);
		_maxGreyValue = std::stoi(line);

		_matrixInMemory = new uint32_t[ _nrow * _ncol ];
	
		line.clear();
		for(size_t i = 0; i < _nrow; i++){	
			std::getline(_matrixFile, line);
		
			for(size_t j = 0; j < _ncol; j++){
				if( line.length() == 0 ){
					//El formato P2 no permite que las lineas de la matriz
					//tengan mas de 70 caracteres. Esto implica que una
					//linea de la imagen esté en más de una línea de la
					//matriz.
					std::getline(_matrixFile, line);
				}
			
				index = line.find(_separador);
				std::stringstream ssLine( line.substr(0, index)  );
				uint32_t value;
				ssLine >> value;

				_matrixInMemory[i*_ncol + j] = value;
		
				line.erase(0, index + _separador.length());	
			}
		}

		_matrixFile.close();
	}
	
public:
	void invert() const
	{
		for (uint32_t i = 0; i < this->rows(); i++) {
			for (uint32_t j = 0; j <this->cols(); j++) {
				uint32_t grayLevel = this->value(i,j);
			
				uint32_t grayLevelInv = _maxGreyValue - grayLevel;
				this->value(i,j, grayLevelInv);

			}
		}
	}
	
	static double similarity(const PGM& pgm1, const PGM& pgm2)
	{
		//PGM similarity(pgm1.rows(), pgm1.cols());
		
		double similarity = 0.0;
		
		if( (pgm1.rows() == pgm2.rows()) && (pgm1.cols() == pgm2.cols()) ){
			uint32_t similarityValue = 0;
			//uint32_t nonZeroPixel = 0;
			for (uint32_t i = 0; i < pgm1.rows(); i++) {
				for (uint32_t j = 0; j < pgm1.cols(); j++) {
					uint32_t pxValue1 = pgm1.value(i,j);
					uint32_t pxValue2 = pgm2.value(i,j);
				
					uint32_t normValue = std::abs((int)pxValue1 - (int)pxValue2);
					//similarity.value(i,j, normValue);
				
					//if(pxValue1 < pgm1.maxGreyValue() && pxValue2 < pgm2.maxGreyValue() ){
						similarityValue += normValue*normValue;
					//	nonZeroPixel++;
					//}

				}
			}
			//std::cout << "nonZeroPixel: " << nonZeroPixel << std::endl;
			similarity =  std::sqrt( (double)similarityValue  / ( pgm1.rows() *  pgm1.cols()));
		}
		else{
			similarity = 10000;
		}
		
		
		return(similarity);
		//return( std::sqrt( (double)similarityValue  / ( pgm1.rows() *  pgm1.cols())) );
		
	}
	
	static double similarity_KL( PGM pgm1,  PGM pgm2)
	{
		double simValue = 0.0;
		
		auto sumPGM = [] (const PGM& pgmMatrix){
			uint32_t sum = 0;
			for(uint32_t i = 0; i < pgmMatrix.rows(); i++){
				for(uint32_t j = 0; j < pgmMatrix.cols(); j++){
					sum +=  pgmMatrix.value(i,j);
				}
			}
			
			return(sum);
		};
		
		//pgm1.invert();
		//pgm2.invert();
		
		//std::cout << sumPGM(pgm1) << std::endl;
		//std::cout << sumPGM(pgm2) << std::endl;
		
		//double pgm1_sum = sumPGM(pgm1);
		//double pgm1_log = std::log(sumPGM(pgm1));
		//double pgm2_log = std::log(sumPGM(pgm2));
		
		double pgm1_pgm2_log = std::log( (double)sumPGM(pgm1) / (double)sumPGM(pgm2) ); 
		
		//std::cout << "--- " <<pgm1_pgm2_log << std::endl;
		
		for(uint32_t i = 0; i < pgm1.rows(); i++){
			for(uint32_t j = 0; j < pgm1.cols(); j++){
				//std::cout << (int)pgm1.value(i,j) - (int)pgm2.value(i,j) << std::endl;
				//simValue +=  pgm1.value(i,j)*(std::log((double)pgm1.value(i,j) / (double)pgm2.value(i,j)) + pgm2_log - pgm1_log);
				//simValue +=  pgm1.value(i,j)*( std::log((double)pgm1.value(i,j) / (double)pgm2.value(i,j)) - pgm1_pgm2_log );
				if( pgm1.value(i,j)*pgm2.value(i,j) > 0){
					simValue +=  pgm1.value(i,j) *( std::log((double)pgm1.value(i,j) / (double)pgm2.value(i,j)) - pgm1_pgm2_log );
				}
			}
		}
		
		
		
		return(simValue);
	}
	
	static double similarity_KL2( PGM pgm1,  PGM pgm2)
	{
		double simValue = 0.0;
		
		/*auto sumPGM = [] (const PGM& pgmMatrix){
			uint32_t sum = 0;
			for(uint32_t i = 0; i < pgmMatrix.rows(); i++){
				for(uint32_t j = 0; j < pgmMatrix.cols(); j++){
					sum +=  pgmMatrix.value(i,j);
				}
			}
			
			return(sum);
		};*/
		
		//pgm1.invert();
		//pgm2.invert();
		
		uint32_t maxGreyValue = pgm1.maxGreyValue();
		
		double* pgm1prob{ new double[maxGreyValue+1]{} };
		double* pgm2prob{ new double[maxGreyValue+1]{} };
		double* pgm12prob{ new double[maxGreyValue+1]{} };

		/*for(uint32_t i = 0; i < maxGreyValue+1; i++){
			pgm1prob[i] = 0.0;
			pgm2prob[i] = 0.0;
			pgm12prob[i] = 0.0;
		}*/
		
		for(uint32_t i = 0; i < pgm1.rows(); i++){
			for(uint32_t j = 0; j < pgm1.cols(); j++){
				pgm1prob[pgm1.value(i,j)] += 1.0 / (double)(pgm1.rows()*pgm1.cols());
				pgm2prob[pgm2.value(i,j)] += 1.0 / (double)(pgm1.rows()*pgm1.cols());
				pgm12prob[pgm1.value(i,j)] += 1.0 / (double)(2*pgm1.rows()*pgm1.cols());
				pgm12prob[pgm2.value(i,j)] += 1.0 / (double)(2*pgm1.rows()*pgm1.cols());
			}
		}
		
		for(uint32_t v = 0; v <  maxGreyValue+1; v++){
			//simValue +=  pgm1.value(i,j) *( std::log((double)pgm1.value(i,j) / (double)pgm2.value(i,j)) - pgm1_pgm2_log );
			
			if( pgm1prob[v]*pgm2prob[v] > 0){
				simValue += pgm1prob[v] * std::log( pgm1prob[v] / pgm2prob[v] );
			}
			
		}

		return(simValue);
	}
	
	static double similarity_entropy( PGM pgm1,  PGM pgm2)
	{
		double simValue = 0.0;
		
		double joinEntropy = 0.0;
		double PaEntropy   = 0.0;
		double PbEntropy   = 0.0;

		
		// crear las frecuencias para cada valor de la imagen:
		
		uint32_t maxGreyValue = pgm1.maxGreyValue();
		
		double* pgm1prob{ new double[maxGreyValue+1]{} };
		double* pgm2prob{ new double[maxGreyValue+1]{} };
		double* pgm12prob{ new double[maxGreyValue+1]{} };
		
		
		

		/*for(uint32_t i = 0; i < maxGreyValue+1; i++){
			//std::cout << "pgm12prob[" << i << "]=" << pgm1prob[i]<< std::endl;
			pgm1prob[i] = 0.0;
			pgm2prob[i] = 0.0;
			pgm12prob[i] = 0.0;
		}*/
		
		for(uint32_t i = 0; i < pgm1.rows(); i++){
			for(uint32_t j = 0; j < pgm1.cols(); j++){
				pgm1prob[pgm1.value(i,j)] += 1.0 / (double)(pgm1.rows()*pgm1.cols());
				pgm2prob[pgm2.value(i,j)] += 1.0 / (double)(pgm1.rows()*pgm1.cols());
				pgm12prob[pgm1.value(i,j)] += 1.0 / (double)(2*pgm1.rows()*pgm1.cols());
				pgm12prob[pgm2.value(i,j)] += 1.0 / (double)(2*pgm1.rows()*pgm1.cols());
			}
		}
		
		
		
		for(uint32_t v = 0; v <  maxGreyValue+1; v++){
			double Pa_v = pgm1prob[v];
			double Pb_v = pgm2prob[v];
			double Pab_v = pgm12prob[v];
	
	
			//std::cout << " a_ij:" << pgm1.value(i,j) << ",  b_ij:" << pgm2.value(i,j) << std::endl;
			//std::cout << "Pa_ij:" << Pa_ij << ", Pb_ij:" << Pb_ij << std::endl;
	
			if(Pab_v > 0){
				joinEntropy += -1 * Pab_v * std::log(Pab_v);
				//joinEntropy += -1 * pgm1.value(i,j)  * std::log(Pb_ij);
			}
			if(Pa_v > 0){
				PaEntropy   += -1 * Pa_v * std::log(Pa_v);
				//PaEntropy   += -1 * pgm1.value(i,j) * std::log(Pa_ij);
			}
			if(Pb_v > 0){
				PbEntropy   += -1 * Pb_v * std::log(Pb_v);
				//PbEntropy   += -1 * pgm2.value(i,j) * std::log(Pb_ij);
			}
		}
		
		//std::cout << "joinEntropy:" << joinEntropy << ", PaEntropy:" << PaEntropy << " ,PbEntropy:" << PbEntropy <<std::endl;
		
		
		simValue = 2 * joinEntropy - PaEntropy - PbEntropy;
		
		simValue /= joinEntropy;
		
		delete[] pgm1prob; 
		delete[] pgm2prob; 
		delete[] pgm12prob;
		
		return(simValue);
	}
	
};














