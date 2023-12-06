#include <yarp/os/all.h>
#include <event-driven/algs.h>
#include <event-driven/vis.h>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace yarp::os;

#include "affine.h"

class objectPos: public yarp::os::RFModule
{
private:

    cv::Size img_size;
    double period{0.01};
    double eros_k, eros_d;
    std::string filename; 
    double rotation{3};
    double u, v, theta, phi, radius;
    
    bool fast = true;

    double r{0.5};
    double begin_time;

    double a, b, c, d, e, f, g, h;

    int user = 5;
    int eye = 0;

    std::ofstream file;

    cv::Mat centre;

    ev::window<ev::AE> input_port;

    ev::EROS eros; 
    cv::Mat eros_conv;
    eyeTracking tracker_handler;
    std::thread computation_thread;

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {
    
        eros_k = rf.check("eros_k", Value(15)).asInt32();
        eros_d = rf.check("eros_d", Value(0.6)).asFloat64();
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
        begin_time = user_params.find("begin").asFloat64();

        eros.init(img_size.width, img_size.height, eros_k, eros_d);

        if (!input_port.open("/shape-position/AE:i")){
            yError()<<"cannot open input port";
            return false;
        }
        file.open("/usr/local/src/workbook_yvonne-vullers/ellipse_code/center.csv");

        if(!file.is_open())
        {
            std::cout << " not check" << std::endl;
        }

        file << std::setprecision(6) << std::fixed;


        yarp::os::Network::connect("/file/ch0dvs:o", "/shape-position/AE:i", "fast_tcp");

        // tracker_handler.init(translation, angle, pscale, nscale);    // KEEP
        tracker_handler.init(u,v,theta, phi, radius, r, rotation, fast); 
    
        computation_thread = std::thread([this]{fixed_step_loop();});
        

        return true;
    }

    double getPeriod() override{
        return period;
    }

    void fixed_step_loop() {

        double timer = 0;
        while (!input_port.isStopping()) {

                ev::info my_info = input_port.readChunkT(0.004, true); //multiple of 4
                timer = my_info.timestamp;
                // std::cout << "timestamp video: " << timer << std::endl;
                
                if (timer > begin_time){
                    for (auto &v : input_port)
                        if(v.y > a*pow(v.x,4)+b*pow(v.x,3)+c*pow(v.x,2)+d*v.x+e && v.y < f*pow(v.x,2)+g*v.x+h){
                            eros.update(v.x, v.y);
                        }


                    if (fast == false){
                        centre = cv::Mat::zeros(260,346, CV_64F);
                        centre.at<double>(u,v) = 1;
                        centre.at<double>(u+1,v) = 1;
                        centre.at<double>(u-1,v) = 1;
                        centre.at<double>(u,v+1) = 1;
                        centre.at<double>(u,v-1) = 1;

                        centre.at<double>(130,173) = 1;
                        centre.at<double>(130+1,173) = 1;
                        centre.at<double>(130-1,173) = 1;
                        centre.at<double>(130,173+1) = 1;
                        centre.at<double>(130,173-1) = 1;
                    }
                        

                                

                    tracker_handler.createTemplates(5);

                    
                    file << timer << " " << float(tracker_handler.ymin) << " " << float(tracker_handler.xmin) << " " << float(tracker_handler.ymax) << " " << float(tracker_handler.xmax) << " " << float(1.0) << std::endl; //<< " " << tracker_handler.center_y << " " << tracker_handler.center_x << " " << tracker_handler.no_motion << std::endl;

                    // tracker_handler.eyeCenter();
                    //std::cout << "created templates" << std::endl;
                    tracker_handler.setEROS(eros.getSurface()); // filter eros and select shape according to ROI. KEEP
                    //tracker_handler.sobelEdges();
                    //std::cout << "set EROS" << std::endl;
                    tracker_handler.performComparisons();                    // for all filters, check similarity score. KEEP
                    //std::cout << "compared" << std::endl;
                    tracker_handler.updateState();                        // change states based on results. NOT NEEDED?
                    //std::cout << "updated" << std::endl;
                    tracker_handler.reset(); 
                    //std::cout << "reset" << std::endl;
                    // yInfo()<<tracker_handler.scores_vector[0]<<tracker_handler.scores_vector[1]<<tracker_handler.scores_vector[2]<<tracker_handler.scores_vector[3] << tracker_handler.scores_vector[4];


                    // cv::Mat norm_mexican;
                    // cv::normalize(tracker_handler.mexican_template_64f, norm_mexican, 1, 0, cv::NORM_MINMAX);
                    // imshow("MEXICAN ROI", tracker_handler.mexican_template_64f+0.5);
                    // imshow("TEMPLATE ROI", tracker_handler.roi_template_64f);
                    // imshow("TEMPLATE RESIZE", tracker_handler.roi_resized);
                    // imshow("EROS ROI", tracker_handler.eros_tracked_64f);
                    //cv::imshow("EROS RESIZE", tracker_handler.eros_resized);

                    //cv::Mat arc = cv::Mat::zeros(260, 346, CV_32F);

                    
        
                    //cv::ellipse(arc,cv::Point(u-30,v-10),cv::Size(80,40),20,-180,0,cv::Scalar(255,255,255),2);
                    //cv::ellipse(arc,cv::Point(145,255),cv::Size(300,80),3, -105,-60,cv::Scalar(255,255,255),4);


                    //arc.convertTo(arc, CV_64F);
                    


                    eros.getSurface().convertTo(eros_conv, CV_64F, 0.003921569); 

                    cv::namedWindow("EROS FULL", cv::WINDOW_NORMAL);
                    cv::imshow("EROS FULL", eros_conv + tracker_handler.current_template + tracker_handler.centers);

                    cv::waitKey(1);

                    
                    // cv::circle(eros_filtered, new_position, 2, 255, -1);
                    // cv::rectangle(eros_filtered, roi_around_shape, 255,1,8,0);
                    // imshow("EROS FULL", tracker_handler.eros_filtered);
                }
        }
    }

    bool updateModule(){
        return true;
    }

    bool interruptModule() override {
        input_port.stop();
        return true;
    }

    bool close() override {
        input_port.stop();

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
    objectPos objectpos;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return objectpos.runModule(rf);
}