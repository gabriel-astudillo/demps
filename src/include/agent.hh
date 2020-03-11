#ifndef _AGENT_HH_
#define _AGENT_HH_
#include <glob.hh>

class Environment;

class Agent {
public:
	typedef std::vector<Agent*> Neighbors;
	static std::shared_ptr<Environment> _myEnv;

	std::list<Point2D> _route; // Primer intento. Debe ser un atributo privado.

private:
	//std::random_device _randomDevice;
	
	uint32_t _id;
	
	struct s_initSpeedRange{
		double   min;
		double   max;
	} _initSpeedRange;
	

	model_t  _model;

	// Para los agentes que conocer donde su meta
	Point2D  _targetPos;

	uint32_t _quad;
	
	Neighbors _agentNeighbors;
	
	Point2D  _position;
	Vector2D _direction;
	Vector2D _currVelocity;

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
	
			return( g_currTimeSim + expoDistro(_randomDevice) ); //
	
		}	
	} _expo;
	
	
	double _responseTime; // Tiempo de respuesta del agente.
	
	//Distribución Rayleigh relacionada
	//con el tiempo de la fase de respuesta.
	double rayleighDistroNumber(double sigma, double tau)
	{
		std::random_device device;
		std::uniform_real_distribution<> rayleighNumber(0.0, 1.0);

		double number = rayleighNumber(device);

		return( tau + sigma * sqrt(-2.0*log(number)) );
	}

public:
	Agent(void);
	Agent(const uint32_t&, const Point2D&, const model_t&, const json& initSpeedRange, const json& phoneUse, const json& SocialForceModel, const json& responseTime);
	//Agent& operator=(const Agent&);

	~Agent(void);

	void          setTargetPos(const Point2D& tposition);
	const Point2D getTargetPos(void) const;

	const Point2D  position(void) const;
	const Vector2D currVelocity(void);
	void           currVelocity(const Vector2D& velocity);

	void     showPosition();
	uint32_t determineQuad();
	void     setQuad();
	void     setQuad(uint32_t idQuad);
	uint32_t getQuad() const;
	void     updateQuad();
	Vector2D direction(void) const;
	void     direction(const Vector2D& direction);

	uint32_t id(void) const;
	model_t model(void) const;
	double  responseTime(void) const;

	void update();

	void shortestPath();
	void followPath();
	void randomWalkway();
	void followTheCrowd(const Neighbors&);
	void followTheCrowd();
	void randomWalkwayForAdjustInitialPosition();

	double distanceTo(Agent* _agent) const;
	

	/*void   setLambda(double L);
	double getLambda();
	
	void    setUsingPhone(uint16_t u);*/
	
	void   setNextTimeUsePhone();
	
	double getNextTimeUsePhone();
	double getProbUsePhone();
	uint16_t getUsingPhone();
	

	
	int getAgentNeighborsSize();
	
};

#endif
