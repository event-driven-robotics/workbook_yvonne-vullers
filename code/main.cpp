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
    double translation{1}, angle{1}, pscale{1.001}, nscale{0.995};
    bool run{false};
    double dt_warpings{0}, dt_comparison{0}, dt_eros{0}, toc_count{0};

    EROSfromYARP eros_handler;
    affineTransforms affine_handler;

    std::thread computation_thread;

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
        affine_handler.cam[affineTransforms::w] = intrinsic_parameters.find("w").asInt32();
        affine_handler.cam[affineTransforms::h] = intrinsic_parameters.find("h").asInt32();
        affine_handler.cam[affineTransforms::cx] = intrinsic_parameters.find("cx").asFloat32();
        affine_handler.cam[affineTransforms::cy] = intrinsic_parameters.find("cy").asFloat32();
        affine_handler.cam[affineTransforms::fx] = intrinsic_parameters.find("fx").asFloat32();
        affine_handler.cam[affineTransforms::fy] = intrinsic_parameters.find("fy").asFloat32();
        img_size = cv::Size(affine_handler.cam[affineTransforms::w], affine_handler.cam[affineTransforms::h]);

        yInfo() << "Camera Size:" << img_size.width << "x" << img_size.height;

        // cv::namedWindow("EROS FULL", cv::WINDOW_NORMAL);
        // cv::resizeWindow("EROS_FULL", img_size);
        // cv::moveWindow("EROS FULL", 0, 0);

        if (!eros_handler.start(img_size, "/atis3/AE:o", getName("/AE:i"), eros_k, eros_d)) {
            yError() << "could not open the YARP eros handler";
            return false;
        }
        yInfo()<<"eros started"; 

        //eros_handler.eros_update_roi=cv::Rect(0,0,640,480);

        affine_handler.init(translation, angle, pscale, nscale);    // KEEP
        affine_handler.initState();                                 // KEEP
        affine_handler.loadTemplate(img_size, filename);            // load star and apply filters. Initialize to center of frame. CHANGE TO CREATE ELLIPSE? OR NOT NEEDED
        yInfo() << img_size.width;
        // affine_handler.createStaticTemplate(img_size, 30);
        // affine_handler.triangleTemplate(img_size); 
        affine_handler.createAffines(translation, cv::Point(img_size.width/2,img_size.height/2), angle, pscale, nscale); // create rotation matrices, NOT NEEDED?
        affine_handler.create_maps();   // ?? 

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
            imshow("EROS FULL", affine_handler.eros_filtered+affine_handler.rot_scaled_tr_template);
            
        }
        else{
            imshow("EROS FULL", eros_handler.eros.getSurface()+affine_handler.initial_template);
        }

        int c = cv::waitKey(1);

        if (c == 32)
            affine_handler.initState();
        if (c == 'g')
            run = true;

        // yInfo()<<affine_handler.state[0]<<affine_handler.state[1]<<affine_handler.state[2]<<affine_handler.state[3];

        return true;
    }

    void tracking_loop() {

        while (!isStopping()) {

            if (run){
                affine_handler.createDynamicTemplate();     // update template of current position. JUST NEED TO CREATE ELLIPSE BASED ON CURRENT PHI AND THETA
                affine_handler.updateAffines();             // update templates wrt rotation and scaling based on new position. NOT NEEDED?
                affine_handler.setROI();                    // select ROI. KEEP
                affine_handler.createMapWarpings();         // create templates. KEEP BUT CHANGE TO ELLIPSES
                // affine_handler.createWarpings(); 
                affine_handler.setEROS(eros_handler.eros.getSurface()); // filter eros and select shape according to ROI. KEEP
                affine_handler.performComparisons();                    // for all filters, check similarity score. KEEP
                affine_handler.updateStateAll();                        // change states based on results. NOT NEEDED?
                //eros_handler.eros_update_roi = affine_handler.roi_around_shape; // update EROS only in ROI 
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
    rf.setDefaultConfigFile("/usr/local/src/workbook_yvonne-vullers/code/config.ini");
    rf.setVerbose(false);
    rf.configure(argc, argv);

    /* create the module */
    affineTracking affinetracking;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return affinetracking.runModule(rf);
}