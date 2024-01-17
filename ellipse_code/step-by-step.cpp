#include <yarp/os/all.h>
#include <event-driven/algs.h>
#include <event-driven/vis.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>

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
    double begin_time, end_time;

    double a, b, c, d, e, f, g, h;

    int user = 13;
    int eye = 1;
    bool use_eros = true;
    bool colored = false;
    bool video = false;

    std::ofstream file;

    cv::Mat centre;

    ev::window<ev::AE> input_port;

    ev::EROS eros; 
    cv::Mat eros_conv;
    eyeTracking tracker_handler;
    std::thread computation_thread;
    cv::Mat window;
    cv::Mat eros_color;
    cv::VideoWriter output;

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
        end_time = user_params.find("end").asFloat64();

        eros.init(img_size.width, img_size.height, eros_k, eros_d);

        if (!input_port.open("/shape-position/AE:i")){
            yError()<<"cannot open input port";
            return false;
        }

        file.open("/usr/local/src/workbook_yvonne-vullers/ellipse_code/center.csv");

        if(!file.is_open())
        {
            std::cout << "File is not opened" << std::endl;
        }

        file << std::setprecision(6) << std::fixed;

        yarp::os::Network::connect("/file/ch0dvs:o", "/shape-position/AE:i", "fast_tcp");

        tracker_handler.init(u,v,theta, phi, radius, r, rotation, fast); 
    
        computation_thread = std::thread([this]{fixed_step_loop();});
        
        if (video){
            output = cv::VideoWriter("/usr/local/src/workbook_yvonne-vullers/ellipse_code/output" + std::to_string(user) + std::to_string(eye) +"eroscf.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 250, cv::Size(img_size.width, img_size.height));
        }

        return true;
    }

    double getPeriod() override{
        return period;
    }

    void fixed_step_loop() {

        double timer = 0;
        double chunk_time = 0.004;
        ev::info my_info;
        
        while (!input_port.isStopping()) {
            
            if (use_eros == true){
                my_info = input_port.readChunkT(0.004, true);
            } else {
                my_info = input_port.readSlidingWinT(0.004, chunk_time); 
                chunk_time += 0.004;
                window = cv::Mat::zeros(img_size.height, img_size.width, CV_32F);
            }
            
            timer = my_info.timestamp;
            
            // std::cout << "timestamp video: " << timer << " " <<my_info.count <<" " << my_info.duration/12.5 <<std::endl;
            
            for (auto &v : input_port)
                // if(v.y > a*pow(v.x,4)+b*pow(v.x,3)+c*pow(v.x,2)+d*v.x+e && v.y < f*pow(v.x,2)+g*v.x+h){

                    if (use_eros == true){
                        eros.update(v.x, v.y); 
                    } else {
                        eros.update(v.x, v.y); 
                        window.at<float>(v.y, v.x) = 1;
                    }
                    
                // }
                         
            if (timer > begin_time){    

                auto tstart = std::chrono::high_resolution_clock::now();

                tracker_handler.createTemplates(5);
                
                if (use_eros == true){
                    tracker_handler.setEROS(eros.getSurface());
                } else {
                    tracker_handler.setEROS(window);
                }

                tracker_handler.performComparisons();                   
                tracker_handler.updateState();  
                tracker_handler.reset(); 

                
                auto tend = std::chrono::high_resolution_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(tend - tstart).count();

                //file << timer << " " << float(tracker_handler.ymin) << " " << float(tracker_handler.xmin) << " " << float(tracker_handler.ymax) << " " << float(tracker_handler.xmax) << " " << float(1.0) << std::endl;
                file << timer << " " << float(tracker_handler.ymin) << " " << float(tracker_handler.xmin) << " " << float(tracker_handler.ymax) << " " << float(tracker_handler.xmax) << " " << float(1.0) << " " << tracker_handler.center_y << " " << tracker_handler.center_x << " " << elapsed_time << std::endl;


            }

            cv::Mat current_template_temp, eros_temp;

            if (use_eros == true){
                eros.getSurface().convertTo(eros_conv, CV_64F, 0.003921569);
                cv::namedWindow("EROS FULL", cv::WINDOW_NORMAL);

                cv::arrowedLine(eros_conv, cv::Point(tracker_handler.center_x,tracker_handler.center_y), cv::Point(tracker_handler.end_x, tracker_handler.end_y),cv::Scalar(255,255,255), 2);
                

                if (colored == true){
                    
                    tracker_handler.current_template.convertTo(current_template_temp, CV_8UC3, 2.5);
                    eros_conv.convertTo(eros_temp, CV_8UC3, 255);
                    cv::cvtColor(eros_temp, eros_temp, cv::COLOR_GRAY2BGR);

                    cv::Mat red = cv::Mat::zeros(tracker_handler.cam[eyeTracking::h], tracker_handler.cam[eyeTracking::w], CV_8UC3);

                    red.setTo(cv::Scalar(0,0,255), current_template_temp);

                    cv::imshow("EROS FULL",  eros_temp + red);
                }


                tracker_handler.current_template.convertTo(current_template_temp, CV_64F, 2);
                cv::imshow("EROS FULL", eros_conv + current_template_temp);
                
        
                
            } else {
                cv::namedWindow("window", cv::WINDOW_NORMAL);
                cv::imshow("window", window);

                eros.getSurface().convertTo(eros_conv, CV_64F, 0.003921569); 
                cv::namedWindow("EROS FULL", cv::WINDOW_NORMAL);
                tracker_handler.current_template.convertTo(current_template_temp, CV_64F, 2);
                cv::imshow("EROS FULL", eros_conv + current_template_temp);
            }

            if (video == true){
                cv::Mat frame = eros_conv + current_template_temp;
            
                frame.convertTo(frame, CV_8U, 255);
                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
                output.write(frame);
            }
                
            if (timer > end_time){
                // cv::waitKey(0);
                std::cout << "done" << std::endl;
                break;
            } else {
                cv::waitKey(1);
            }
            
    
        }
    }

    bool updateModule(){
        return true;
    }

    bool interruptModule() override {
        if (video == true){
            output.release();
        }
        
        input_port.stop();
        return true;
    }

    bool close() override {
        if (video == true){
            output.release();
        }

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