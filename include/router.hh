#ifndef _ROUTER_H_
#define _ROUTER_H_
#include <glob.hh>

class Router
{

private:
    osrm::EngineConfig          _config;
    std::shared_ptr<osrm::OSRM> _osrm;
    LocalCartesian              _projector;

public:
    class Response
    {
    private:
        double _distance;
        double _duration;
        std::list<Point2D> _path;
    public:
        Response(void)
        {
            ;
        }
        Response(const double &_distance,const double &_duration,const std::list<Point2D> _path)
        {
            this->_distance=_distance;
            this->_duration=_duration;
            this->_path=_path;
        }
        Response(const Response &_r)
        {
            this->_distance=_r._distance;
            this->_duration=_r._duration;
            this->_path=_r._path;
        }
        Response& operator=(const Response &_r)
        {
            this->_distance=_r._distance;
            this->_duration=_r._duration;
            this->_path=_r._path;
            return(*this);
        }
        ~Response(void)
        {
            this->_path.clear();
        }
        double distance(void) const
        {
            return(this->_distance);
        }
        double duration(void) const
        {
            return(this->_duration);
        }
        std::list<Point2D> path(void) const
        {
            return(this->_path);
        }
    };

    Router(void);
    Router(const Router&);
    Router(const json&,const std::string&);
    ~Router(void);

    Router& operator=(const Router&);

    Response route(const Point2D&,const Point2D&);
    Response route(const Point2D&,const double&);

};
#endif
