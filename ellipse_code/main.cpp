#include <yarp/os/all.h>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace yarp::os;

#include "eros.h"
#include "affine.h"

class affineTracking: public yarp::os::RFModule
{
private:

    cv::Size img_size;
    double period{0.01};
    double eros_k, eros_d;
    std::string filename; 
    //double translation{1}, angle{1}, pscale{1.001}, nscale{0.995};
    double rotation{M_PI/100};
    double u, v, theta, phi, radius;
    double r{0.55};
    
    bool run{false};
    double dt_warpings{0}, dt_comparison{0}, dt_eros{0}, toc_count{0};

    EROSfromYARP eros_handler;
    eyeTracking tracker_handler;

    std::thread computation_thread;
    cv::Mat eros_conv, centre;

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        // options and parameters
    
        eros_k = rf.check("eros_k", Value(9)).asInt32();
        eros_d = rf.check("eros_d", Value(0.5)).asFloat64();
        period = rf.check("period", Value(0.01)).asFloat64();
        filename = rf.check("shape-file", Value("/usr/local/src/workbook_yvonne-vullers/code/star.png")).asString(); 

        // module name
        setName((rf.check("name", Value("/shape-position")).asString()).c_str());

        yarp::os::Bottle& intrinsic_parameters = rf.findGroup("CAMERA_CALIBRATION");
        if (intrinsic_parameters.isNull()) {
            yError() << "Wrong .ini file or [CAMERA_CALIBRATION] not present. Deal breaker.";
            return false;
        }
        tracker_handler.cam[eyeTracking::w] = intrinsic_parameters.find("w").asInt32();
        tracker_handler.cam[eyeTracking::h] = intrinsic_parameters.find("h").asInt32();
        tracker_handler.cam[eyeTracking::cx] = intrinsic_parameters.find("cx").asFloat32();
        tracker_handler.cam[eyeTracking::cy] = intrinsic_parameters.find("cy").asFloat32();
        tracker_handler.cam[eyeTracking::fx] = intrinsic_parameters.find("fx").asFloat32();
        tracker_handler.cam[eyeTracking::fy] = intrinsic_parameters.find("fy").asFloat32();
        img_size = cv::Size(tracker_handler.cam[eyeTracking::w], tracker_handler.cam[eyeTracking::h]);

        yInfo() << "Camera Size:" << img_size.width << "x" << img_size.height;

        cv::namedWindow("EROS FULL", cv::WINDOW_NORMAL);
        cv::resizeWindow("EROS_FULL", img_size);
        cv::moveWindow("EROS FULL", 0, 0);



        if (!eros_handler.start(img_size, "/file6/ch0dvs:o", getName("/AEa:i"), eros_k, eros_d)) {
            yError() << "could not open the YARP eros handler";
            return false;
        }
        yInfo()<<"eros started"; 

        phi = M_PI/2;
        theta = -M_PI/3;
        radius = 90; //125; //98
        u = 215; //190;  //90
        v = 185; //125;  //37

        centre = cv::Mat::zeros(260,346, CV_64F);
        centre.at<double>(u,v) = 1;
        centre.at<double>(u+1,v) = 1;
        centre.at<double>(u-1,v) = 1;
        centre.at<double>(u,v+1) = 1;
        centre.at<double>(u,v-1) = 1;



        // tracker_handler.init(translation, angle, pscale, nscale);    // KEEP
        tracker_handler.init(u,v,theta, phi, radius, r, rotation);                                 // KEEP
        computation_thread = std::thread([this]{tracking_loop();});

        if(!run) yInfo() << "WARNING: press G to start tracking (--run)";

        return true;
    }

    double getPeriod() override{
        return period;
    }
        
    bool updateModule() {
        cv::Mat norm_mexican;
        if (run){
            // std::cout << tracker_handler.eros_filtered.size() << std::endl;
            // std::cout << tracker_handler.current_template.size() << std::endl;
            cv::imshow("EROS FULL", tracker_handler.eros_tracked_64f + tracker_handler.current_template +tracker_handler.rectangle_eros);
           
        }
        else{
            eros_handler.eros.getSurface().convertTo(eros_conv, CV_64F, 0.003921569); 
            cv::imshow("EROS FULL", eros_conv + tracker_handler.current_template + centre);
        }

        

        int c = cv::waitKey(1);

        if (c == 32)
            tracker_handler.init(u,v,theta, phi, radius, r, rotation);
        if (c == 'g')
            run = true;

        // yInfo()<<tracker_handler.state[0]<<tracker_handler.state[1]<<tracker_handler.state[2]<<tracker_handler.state[3];

        return true;
    }

    void tracking_loop() {

        while (!isStopping()) {

            if (run){
                //std::cout << "running" << std::endl;
                // 1) update the templates according to the current state +- a little change 
                tracker_handler.createTemplates(5);
                //std::cout << "created templates" << std::endl;
                tracker_handler.setEROS(eros_handler.eros.getSurface()); // filter eros and select shape according to ROI. KEEP
                //std::cout << "set EROS" << std::endl;
                tracker_handler.performComparisons();                    // for all filters, check similarity score. KEEP
                //std::cout << "compared" << std::endl;
                tracker_handler.updateStateAll();                        // change states based on results. NOT NEEDED?
                //std::cout << "updated" << std::endl;
                tracker_handler.reset(); 
                //std::cout << "reset" << std::endl;

            }

        }
    }

    bool interruptModule() override {
        return true;
    }

    bool close() override {

        yInfo() << "waiting for eros handler ... ";
        eros_handler.stop();
        yInfo() << "waiting for computation thread ... ";
        computation_thread.join();

        return true;
    }
};

int main(int argc, char *argv[]) {
    /* initialize yarp network */

    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }


    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("event-driven");
    rf.setDefaultConfigFile("/usr/local/src/workbook_yvonne-vullers/ellipse_code/config.ini");
    rf.setVerbose(false);
    rf.configure(argc, argv);

    /* create the module */
    affineTracking affinetracking;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return affinetracking.runModule(rf);
}