#include <yarp/os/all.h>
#include <event-driven/algs.h>
#include <event-driven/vis.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <string>
#include <vector>

using namespace yarp::os;


ev::window<ev::AE> input_port;
cv::Mat window;
ev::EROS eros; 




int main () {
    yarp::os::Network yarp;

    std::ofstream file;
	file.open("/usr/local/src/workbook_yvonne-vullers/create_ellipse/test.csv");

    if(!file.is_open())
    {
        std::cout << " not check" << std::endl;
    }

    if (!input_port.open("/shape-position/AE:i")){
            yError()<<"cannot open input port";
        }



    yarp::os::Network::connect("/file/ch0dvs:o", "/shape-position/AE:i", "fast_tcp");
    ev::info my_info;
    eros.init(346, 260, 15, 0.6);

    double chunk_time = 4.4;

    while (!input_port.isStopping()) {

        my_info = input_port.readSlidingWinT(0.002, chunk_time); //multiple of 4 ms
        //my_info = input_port.readChunkT(0.004, true);

        std::cout << my_info.timestamp << std::endl;

        chunk_time += 0.002;

        window = cv::Mat::zeros(260, 346, CV_32F);

        for (auto &v : input_port){
            window.at<float>(v.y, v.x) = 1;
            eros.update(v.x, v.y); 
        }

        if (my_info.timestamp > 5.919){
            // file << my_info.timestamp << std::endl;
            file<< cv::format(eros.getSurface(), cv::Formatter::FMT_CSV) << std::endl;
        }
        
        // #cv::imwrite("frame.jpg", eros.getSurface());
        // std::cout << eros.getSurface() << std::endl;

        cv::imshow("WINDOW", eros.getSurface());
        cv::waitKey(0);
    }
    
}