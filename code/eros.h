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
        while (!input_port.isStopping()) {
            ev::info my_info = input_port.readAll(true);
            static double t_eros_start = my_info.timestamp;
            tic = my_info.timestamp;
            dt = tic - t_eros_start;
            dur = my_info.duration; 
            packet_events = my_info.count; 
            events_inside_roi = 0; 
            for(auto &v : input_port){
                if((v.x) > eros_update_roi.x && (v.x) < eros_update_roi.x + eros_update_roi.width && (v.y) > eros_update_roi.y && (v.y) < eros_update_roi.y + eros_update_roi.height){
                    // if (filter.check(v.x, v.y, v.p, my_info.timestamp)){
                        eros.update(v.x, v.y);
                        events_inside_roi++; 
                    // }
                }
            }
            n_events_eros_update = events_inside_roi;
            dt_not_read_events = input_port.stats_unprocessed().duration;
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