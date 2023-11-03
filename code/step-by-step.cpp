#include <yarp/os/all.h>
#include <event-driven/algs.h>
#include <event-driven/vis.h>

using namespace yarp::os;

#include "affine.h"

class objectPos: public yarp::os::RFModule
{
private:

    cv::Size img_size;
    double period{0.01};
    double eros_k, eros_d;
    double tau_latency{0};
    double recording_duration, elapsed_time{0}; 
    double translation{1.0}, angle{1}, pscale{1.001}, nscale{0.999};
    bool run{false};
    double dt_warpings{0}, dt_comparison{0}, dt_eros{0}, toc_count{0};
    std::string filename; 

    ev::window<ev::AE> input_port;

    ev::EROS eros; 
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

        eros.init(img_size.width, img_size.height, eros_k, eros_d);

        if (!input_port.open("/object_position/AE:i")){
            yError()<<"cannot open input port";
            return false;
        }

        yarp::os::Network::connect("/atis3/AE:o", "/object_position/AE:i", "fast_tcp");

        affine_handler.init(translation, angle, pscale, nscale);
        affine_handler.initState();
        // affine_handler.starTemplate(img_size);
        affine_handler.loadTemplate(img_size, filename);
        affine_handler.createAffines(translation, cv::Point(img_size.width/2,img_size.height/2), angle, pscale, nscale);
        affine_handler.create_maps(); 

        computation_thread = std::thread([this]{fixed_step_loop();});

        return true;
    }

    double getPeriod() override{
        return period;
    }

    void fixed_step_loop() {

        while (!input_port.isStopping()) {

                ev::info my_info = input_port.readChunkT(0.001, true);
                for (auto &v : input_port)
                    eros.update(v.x, v.y);

                affine_handler.createDynamicTemplate();
                affine_handler.updateAffines();
                affine_handler.setROI();
                affine_handler.createMapWarpings();
                // affine_handler.createWarpings(); 
                affine_handler.setEROS(eros.getSurface());
                affine_handler.performComparisons();
                affine_handler.updateStateAll();
                yInfo()<<affine_handler.state[0]<<affine_handler.state[1]<<affine_handler.state[2]<<affine_handler.state[3];
                // cv::Mat norm_mexican;
                // cv::normalize(affine_handler.mexican_template_64f, norm_mexican, 1, 0, cv::NORM_MINMAX);
                // imshow("MEXICAN ROI", affine_handler.mexican_template_64f+0.5);
                // imshow("TEMPLATE ROI", affine_handler.roi_template_64f);
                // imshow("TEMPLATE RESIZE", affine_handler.roi_resized);
                // imshow("EROS ROI", affine_handler.eros_tracked_64f);
                imshow("EROS RESIZE", affine_handler.eros_resized);
                imshow("EROS FULL", affine_handler.eros_filtered + affine_handler.rot_scaled_tr_template);
                cv::waitKey(0);
                
                // cv::circle(eros_filtered, new_position, 2, 255, -1);
                // cv::rectangle(eros_filtered, roi_around_shape, 255,1,8,0);
                // imshow("EROS FULL", affine_handler.eros_filtered);

        }
    }

    bool updateModule(){
        return true;
    }

    bool interruptModule() override {
        return true;
    }

    bool close() override {

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
    objectPos objectpos;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return objectpos.runModule(rf);
}