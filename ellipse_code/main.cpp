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
    double rotation{3};
    double u, v, theta, phi, radius;
    double r{0.5};
    
    bool run{false};
    double dt_warpings{0}, dt_comparison{0}, dt_eros{0}, toc_count{0};

    EROSfromYARP eros_handler;
    eyeTracking tracker_handler;

    std::thread computation_thread;
    cv::Mat eros_conv, centre;
    bool fast = true;
    std::ofstream file;

    double a, b, c, d, e, f, g, h;

    int user = 18;
    int eye = 0;

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        // options and parameters
        eros_k = rf.check("eros_k", Value(15)).asInt32();
        eros_d = rf.check("eros_d", Value(0.6)).asFloat64();
        period = rf.check("period", Value(0.01)).asFloat64();


        // module name
        setName((rf.check("name", Value("/shape-position")).asString()).c_str());


        // Set camera parameters
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


        // Set user parameters
        yarp::os::Bottle& user_params = rf.findGroup("USER_" + std::to_string(user) + "_" + std::to_string(eye));

        phi = user_params.find("phi").asFloat64();
        theta = user_params.find("theta").asFloat64();
        radius = user_params.find("radius").asFloat64();
        u = user_params.find("u").asFloat64();
        v = user_params.find("v").asFloat64();
        a = user_params.find("a").asFloat64();
        b = user_params.find("b").asFloat64();
        c = user_params.find("c").asFloat64();
        d = user_params.find("d").asFloat64();
        e = user_params.find("e").asFloat64();
        f = user_params.find("f").asFloat64();
        g = user_params.find("g").asFloat64();
        h = user_params.find("h").asFloat64();
        

        // Start EROS
        cv::namedWindow("EROS FULL", cv::WINDOW_NORMAL);
        cv::resizeWindow("EROS_FULL", img_size);
        cv::moveWindow("EROS FULL", 0, 0);

        if (!eros_handler.start(img_size, "/file/ch0dvs:o", getName("/AE:i"), eros_k, eros_d, a, b, c, d, e, f, g, h)) {
            yError() << "could not open the YARP eros handler";
            return false;
        }
        yInfo()<<"eros started"; 


        // Open file to write results
        file.open("/usr/local/src/workbook_yvonne-vullers/ellipse_code/center.csv");

        if(!file.is_open())
        {
            std::cout << " not check" << std::endl;
        }

        file << std::setprecision(6) << std::fixed;
  

        // Start thread
        tracker_handler.init(u,v,theta, phi, radius, r, rotation, fast);
        computation_thread = std::thread([this]{tracking_loop();});

        if(!run) yInfo() << "WARNING: press G to start tracking (--run)";

        return true;
    }

    double getPeriod() override{
        return period;
    }
        
    bool updateModule() {

        if (run){
            cv::imshow("EROS FULL", tracker_handler.eros_tracked_64f + tracker_handler.current_template);
           
        }
        else{
            // cv::Mat arc = cv::Mat::zeros(260, 346, CV_32F);   
            // cv::ellipse(arc,cv::Point(205,163),cv::Size(90,70),5,180,360,cv::Scalar(255,255,255),2);
            // cv::ellipse(arc,cv::Point(185,90),cv::Size(200,80),10, 40,110,cv::Scalar(255,255,255),2);
            // arc.convertTo(arc, CV_64F);

            eros_handler.eros.getSurface().convertTo(eros_conv, CV_64F, 0.003921569); 
            cv::imshow("EROS FULL", eros_conv + tracker_handler.current_template);
        }

    
        int c = cv::waitKey(1);

        if (c == 32)
            tracker_handler.init(u,v,theta, phi, radius, r, rotation, fast);
        if (c == 'g')
            run = true;

        // yInfo()<<tracker_handler.state[0]<<tracker_handler.state[1]<<tracker_handler.state[2]<<tracker_handler.state[3];

        return true;
    }

    void tracking_loop() {
        double time_prev = 0;
        double timer = 0;

        while (!isStopping()) {

            if (run){
                //std::cout << "running" << std::endl;
                // 1) update the templates according to the current state +- a little change 
                tracker_handler.createTemplates(5);

                // tracker_handler.eyeCenter();

                //file << eros_handler.time << " " << tracker_handler.center_y << " " << tracker_handler.center_x << std::endl;
                timer = eros_handler.time;
                file << timer << " " << float(tracker_handler.ymin) << " " << float(tracker_handler.xmin) << " " << float(tracker_handler.ymax) << " " << float(tracker_handler.xmax) << " " << float(1.0) << " " << tracker_handler.center_y << " " << tracker_handler.center_x << " " << tracker_handler.no_motion << std::endl;


                //std::cout << eros_handler.time << std::endl;
                //std::cout << "created templates" << std::endl;
                tracker_handler.setEROS(eros_handler.eros.getSurface()); // filter eros and select shape according to ROI. KEEP
                //std::cout << "set EROS" << std::endl;
                tracker_handler.performComparisons();                    // for all filters, check similarity score. KEEP
                //std::cout << "compared" << std::endl;
                tracker_handler.updateState();                        // change states based on results. NOT NEEDED?
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