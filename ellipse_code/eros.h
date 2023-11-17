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
    double tic{-1};
    double dur{0};
    double dt{0};
    int packet_events{0};
    int events_inside_roi{0}, n_events_eros_update{0};

    void erosUpdate() 
    {
    double a =  2.1263607525117082e-07 ;
    double b =  -0.00012994337427812272 ;
    double c =  0.03370327571278422 ;
    double d =  -3.9643992828139734 ;
    double e =  214.62571884811135 ;
    double f =  0.0014607785004303083 ;
    double g =  -0.3289652109686471 ;
    double h =  122.1432665611946 ;
        
        while (!input_port.isStopping()) {
            ev::info my_info = input_port.readAll(true);
            for(auto &v : input_port)
                // if(v.y > a*pow(v.x,4)+b*pow(v.x,3)+c*pow(v.x,2)+d*v.x+e+3 && v.y < f*pow(v.x,2)+g*v.x+h-5){
                    eros.update(v.x, v.y);
                // }
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