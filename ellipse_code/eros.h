#pragma once

#include <yarp/os/all.h>
#include <event-driven/algs.h>
#include <event-driven/vis.h>

using namespace yarp::os;

class EROSfromYARP
{
public:

    ev::window<ev::AE> input_port;
    ev::EROS eros;
    std::thread eros_worker;
    cv::Rect eros_update_roi;
    ev::vNoiseFilter filter; 
    double dt_not_read_events{0};
    double time;
    double tic{-1};
    double dur{0};
    double dt{0};
    int packet_events{0};
    int events_inside_roi{0}, n_events_eros_update{0};

    void erosUpdate() 
    {
    double a =  3.092363719928957e-07 ;
    double b =  -0.00013699489513964642 ;
    double c =  0.025347984304222058 ;
    double d =  -2.498065384686753 ;
    double e =  190.17984418965204 ;

    double f =  0.0014907553516399722 ;
    double g =  -0.2717319543680887 ;
    double h =  141.92979888811996 ;

        while (!input_port.isStopping()) {
            ev::info my_info = input_port.readAll(true);
            time = my_info.timestamp;
            //std::cout << "timestamp video: " << my_info.timestamp << std::endl;
            for(auto &v : input_port)
                if(v.y > a*pow(v.x,4)+b*pow(v.x,3)+c*pow(v.x,2)+d*v.x+e && v.y < f*pow(v.x,2)+g*v.x+h){
                    eros.update(v.x, v.y);
                }
        }
        
    }

public:
    bool start(cv::Size resolution, std::string sourcename, std::string portname, int k, double d)
    {
        eros.init(resolution.width, resolution.height, k, d);

        if (!input_port.open(portname))
            return false;

        yarp::os::Network::connect(sourcename, portname, "fast_tcp");

        filter.initialise(resolution.width, resolution.height);
        filter.use_temporal_filter(0.01); 

        eros_worker = std::thread([this]{erosUpdate();});
        return true;
    }

    void stop()
    {
        input_port.stop();
        eros_worker.join();
    }

};