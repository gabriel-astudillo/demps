#pragma once

#include <globals.hh>

#include <random>
#include <json.hpp>
using json=nlohmann::json;

#include <CGAL.hh>

class Environment;
class Zone;

class Agent {
public:
	typedef std::vector<Agent*> Neighbors;
	static std::shared_ptr<Environment> _myEnv;

	std::list<Point2D> _route, _routeOld; // Primer intento. Debe ser un atributo privado.
	
	static struct panicState_s {
		enum code {
			susceptible = 0,
			infected    = 1,
			recovered   = 2
		};
	} panicState;
	typedef panicState_s::code panicState_t;

private:
	//std::random_device _randomDevice;
	
	uint32_t _id;
	
	std::string  _initialZoneNameID;
	model_t  _model;
	double   _radius;
	uint32_t _groupAge;
	double   _density; //Densidad de personas alrededor del agente
	//int32_t  _elevation;
	double   _gradient;
	
	//bool     _newRouteByDebris;
	
	// Datos de la zona segura
	struct s_safeZoneData{
		std::string  safeZoneNameID;
		Zone*        safeZone;
		Point2D      targetPos;
		double       distanceToTargetPos;
		bool         isFake; //false para residentes y visitantes I. true para visitantes II.
	} _safeZoneData;	
	
	struct s_initSpeedRange{
		double   min;
		double   max;
	} _initSpeedRange;
	
	struct s_ageRange{
		
		struct {
			double prob;
			double minSpeed;
			double maxSpeed;
		} groupFeatures[4];
		
		
		void getAgeFeatures(uint32_t& groupAge, double& minSpeed, double& maxSpeed)
		{
			std::random_device device;
			std::mt19937 rng(device());
			std::uniform_real_distribution<double> unif(0.0, 1.0);
			
			double trigg = unif(rng);
			
			double probAcum = 0;
			for(size_t i = 0; i < 4; i++){
				probAcum += groupFeatures[i].prob;
				if(trigg <= probAcum){
					groupAge = i;
					minSpeed = groupFeatures[i].minSpeed;
					maxSpeed = groupFeatures[i].maxSpeed;
					break;
				}
			}

		}
	} _ageRange;
		
	Neighbors _agentNeighbors;

	uint32_t _quad;
	uint32_t _quadOld;
	
	
	Point2D  _position;
	Vector2D _direction;
	Vector2D _currVelocity;
	
	struct s_evacuationData{
		bool   inSafeZone;
		bool   arrivedAtDestinationPoint;
		double evacuationTime;
		double travelDistance;
		
		bool   isWaiting;
		bool   isMoving;
		bool   isAlive;
		bool   isMovingRandomDueDebris;
		bool   isRouteRandom;
		
		int    timeLived;
	} _evacuationData;

	//Variables exclusivas para el modelo de Fuerza social
	//Helbing, D., & Molnar, P. (1998).
	//Social Force Model for Pedestrian Dynamics. Physical Review E, 51(5), 4282–4286.
	struct s_SFM {
		double  disiredSpeed;
		double  maxDisiredSpeed;
	

		//Driving force of the taget Point
		double  timeRelax;

		//Interaction force between agents
		double  sigma;    //[m]
		double  strengthSocialRepulsiveForceAgents;
		double  cosPhi; //cos(200º)
	} _SFM;
	
	
	//Comportamiento agente c/respecto al uso del teléfono 
	struct s_usePhone {
		double nextTimeUsePhone;    //  ==> delta de tiempo futuro cuando usara el telefono
		double probPhoneUse;        //  ==> cuando llegue el tiemmo de usar, lo usara con cierta probabilidad
		double probPhoneUseConst;   //  ==> constante de la exponencial neg. de la probabilidad de uso
		uint8_t usingPhone;    // 0: no; 1: sí.		
	} _usePhone;
	
	//Distribución exponencial relacionada
	//con el tiempo entre uso del teléfono
	struct s_expo {
		double lambda;              //  ==> 1/(tiempo promedio de uso del telefono)
		
		double exponentialTime()
		{
			std::random_device _randomDevice;
			std::exponential_distribution<double> expoDistro(lambda);
	
			return( global::currTimeSim + expoDistro(_randomDevice) ); //
	
		}	
	} _expo;
	
	
	double _responseTime; // Tiempo de respuesta del agente.
	
	//Distribución Rayleigh relacionada
	//con el tiempo de la fase de respuesta.
	
	struct s_responseTime {
		
		double sigma;
		double tau;
		
		double getRayleigh()
		{
			std::random_device device;
			std::uniform_real_distribution<> rayleighNumber(0.0, 1.0);

			double number = rayleighNumber(device);

			return( sigma * sqrt(-2.0*log(number)) );
		}
		
		double getLogNormalNumber(double mean, double std){
			std::random_device device;
			std::lognormal_distribution<double> logNormalDistro(mean,std);
	
			return( logNormalDistro(device) );
		}
	
	} _responseTimeEngine;
	
	struct s_panic{
	
		Agent::panicState_t stateCode;
		double susceptibleTime;
		double infectedTime;
		double recoveredTime;
		
		struct s_params{
			double emotionMinValue;
			double emotionThreshold;
			double probInfectedToRecovered;
			double probRecoveredToSusceptible;
			double meanTimeInInfected;
			double sdTimeInInfected;
			double timeInInfected;
			double meanTimeInRecovered; 
			double sdTimeInRecovered; 
			double timeInRecovered;
		} params;

		struct s_emotion{
			double combined; // Para modelo de calculo de nivel de emocion nro 3.
			double strength;
			double expression;
			double recv;
			double send;
		} emotionOrig, emotion;
		
		
		std::map<panicState_s::code, std::string> codenames = {\
			{Agent::panicState.susceptible, "susceptible"},\
			{Agent::panicState.infected   , "infected"},\
			{Agent::panicState.recovered  , "recovered"}
		};
		
		s_panic(){
			std::random_device device;
			std::uniform_real_distribution<> intensity(0.01, 1.0);
			
			//params      = {0.6, 0.7, 0.3, 300, 300};
			emotionOrig = {0.0, 0.0, intensity(device), intensity(device), intensity(device) };
			emotion     = emotionOrig;
			
			susceptibleTime = 0.0;
			infectedTime  = 0.0;
			recoveredTime = 0.0;
			stateCode = Agent::panicState.susceptible;
		}
		
		std::string stateName(){
			return(codenames[stateCode]);
		}
	} _panic;

public:
	typedef s_panic panic_t;
	typedef s_evacuationData evacuationData_t;
	typedef s_SFM SFM_t;


public:
	Agent(void);
	Agent(const uint32_t&, \
		const Point2D&,\
		const std::string &,\
		const model_t&,\
		const json& ageRange,\
		const json& phoneUse,\
		const json& SocialForceModel,\
		const json& panicModel,\
		const json& responseTime);
	//Agent& operator=(const Agent&);

	~Agent(void);

	void          setTargetPos(const Point2D& tposition); 
	const Point2D getTargetPos(void) const;
	
	void          setSafeZoneID(const std::string& safeZoneNameID);
	std::string   getSafeZoneID();
	std::string   getInitialZoneID();
	
	void          safeZone(Zone* safeZonePtr);
	Zone*         safeZone();
	
	const double  distanceToTargetPos();
	void          distanceToTargetPos(const double& dist);
	
	void          safeZoneDataIsFake(const bool& b);
	bool          safeZoneDataIsFake();

	const Point2D  position(void) const;
	void           position(const Point2D& position);
	const Vector2D currVelocity(void);
	void           currVelocity(const Vector2D& velocity);
	
	bool     inSafeZone();
	void     inSafeZone(bool in);
	void     isMoving(bool m);
	bool     isWaiting();
	void     isWaiting(bool w);
	bool     isMovingRandomDueDebris();
	void     isMovingRandomDueDebris(bool d);
	bool     isRouteRandom();
	void     isRouteRandom(bool r);
	void     isAlive(bool i);
	bool     isAlive();
	
	
	
	void     evacuationTime(uint32_t& currTick);
	//double   evacuationTime();
	//double   travelDistance();

	void     showPosition();
	void     showPanic();
	//uint32_t determineQuad();
	//void     setQuad();
	void     setQuad(uint32_t idQuad);
	uint32_t getQuad() const;
	void     updateQuad();
	void     updateGradient(uint32_t curPatch, uint32_t newPatch);
	
	//void newRouteByDebris(bool d);
	//bool newRouteByDebris();
	
	Vector2D direction(void) const;
	void     direction(const Vector2D& direction);

	uint32_t id(void) const;
	double radius(void) const;
	model_t model(void) const;
	uint32_t groupAge() const;
	double getDensity(void) const;
	void   setDensity(const double& density);
	
	//int32_t getElevation() const;
	//void    setElevation(const int32_t elevation);
	
	double  getGradient() const;
	void    setGradient(const double gradient);
	
	double  responseTime(void) const;

	void update();

	void shortestPath();
	void followPath();
	void randomWalkway();
	void followTheCrowd(const Neighbors&);
	void followTheCrowd();
	void randomWalkwayForAdjustInitialPosition();

	double distanceTo(Agent* _agent) const;
	
	void   setNextTimeUsePhone();
	
	double getNextTimeUsePhone();
	double getProbUsePhone();
	uint16_t getUsingPhone();
	

	void setAgentNeighbors(const Neighbors& n);
	void clearAgentNeighbors();
	void addAgentNeighbors(Agent* ag);
	Neighbors& getAgentNeighbors();
	int  getAgentNeighborsSize();
	
	Agent::panic_t getPanicStruct();
	Agent::evacuationData_t getEvacuationDataStruct();
	void setEvacuationDataStruct(Agent::evacuationData_t eData);
	
	Agent::SFM_t getSFMstruct();
	
	
};

