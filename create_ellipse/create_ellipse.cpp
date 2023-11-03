#include <iostream> 
#include <cmath>

#include <opencv2/imgproc.hpp> 
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 

#include <yarp/os/all.h>

#include <event-driven/algs.h>
#include <event-driven/vis.h>
#include <algorithm>
#include <iterator>
using namespace yarp::os;

cv::Mat makeEllipse (float r, double theta, double phi, int height, int width){
	cv::Mat ell_filter = cv::Mat::zeros(2*height+1, 2*width+1, CV_32F);	// CHECK +1
	double x = 0;
	double y = 0;


	for (float t = -M_PI; t < M_PI; t += 2*M_PI/1000){
		// YAW (y) & PITCH (x)
		x = r*cos(t)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;
		y = r*sin(t)*cos(phi)-sqrt(1 - pow((x-1),2) - pow(r*sin(t),2))*sin(phi) +1;


		//std::cout<<x << ", " << y << std::endl;
		// YAW (Y) & ROLL (z)
		// x = (r*cos(theta)*cos(t) + sqrt(1-pow(r,2))*sin(theta))*cos(phi) - r*sin(t)*sin(phi) + 1;
		// y = (r*cos(theta)*cos(t) + sqrt(1-pow(r,2))*sin(theta))*sin(phi) + r*sin(t)*cos(phi) + 1;

		// std::cout << "x: " <<  x << "  y: " << y << "  t: " << t << std::endl;

		if((t > -M_PI/4 && t < M_PI/4) || (t > 3*M_PI/4 || t < -3*M_PI/4)){
			ell_filter.at<float>(round(y*width),round(x*height)) = 1;
		}
	}

	return ell_filter;
}

void points(){
	cv::Mat arc = cv::Mat::zeros(400,400, CV_32F);

	// cv::ellipse(arc,cv::Point(145,305),cv::Size(140,200),32, -155,-55,cv::Scalar(255,255,255),1); //upper
	cv::ellipse(arc,cv::Point(145,255),cv::Size(300,80),3, -105,-60,cv::Scalar(255,255,255),1); //lower
	// for(int i = 0; i <400; i++){
	// 	for(int j = 0; j < 400; j++){
	// 		if(arc.at<float>(i,j) == 255){
	// 			std::cout << i << ", " ; //<< 399-i << std::endl;
	// 		}
	// 	}
	// }

	cv::namedWindow("circle", cv::WINDOW_NORMAL);
	cv::imshow("circle", arc);
	cv::waitKey(0);
}

void eyelid(int height, int width){
	cv::Mat eyelid_shape = cv::Mat::zeros(2*height+1, 2*width+1, CV_64F);

	double a = 1.51942031*pow(10,-7);
	double b = -1.05979615*pow(10,-4);
	double c = 3.05589958*pow(10,-2);
	double d = -4.36182599*pow(10,0);
	double e = 3.73315976*pow(10,2);

	for(int x = 85; x <313; x++){
		int y = a*pow(x,4)+b*pow(x,3)+c*pow(x,2)+d*x+e;
		eyelid_shape.at<double>(round(y), x) = 1;
	}

	cv::namedWindow("eyelid", cv::WINDOW_NORMAL);
	cv::imshow("eyelid", eyelid_shape);
	cv::waitKey(0);

}

void eyeshape(){
	cv::Mat arc = cv::Mat::zeros(400,400, CV_32F);
	cv::Mat mask = cv::Mat::zeros(400,400, CV_32F);
	cv::Mat masked = cv::Mat::zeros(400,400, CV_32F);
	cv::Mat pupil = cv::Mat::zeros(400,400, CV_32F);
	
	
	
	cv::ellipse(pupil,cv::Point(130,175),cv::Size(50,50),0, 0,360,cv::Scalar(255,255,255),4);

	//cv::ellipse(arc,cv::Point(160,310),cv::Size(150,200),25, -150,-50,cv::Scalar(255,255,255),-1);
	cv::ellipse(arc,cv::Point(145,305),cv::Size(140,200),32, -155,-55,cv::Scalar(255,255,255),-1);
	cv::ellipse(mask,cv::Point(240,60),cv::Size(170,200), 25, 125,35,cv::Scalar(255,255,255),-1);

	cv::bitwise_and(mask, arc, masked);
	cv::bitwise_and(pupil, masked, pupil);

	cv::ellipse(pupil,cv::Point(160,310),cv::Size(150,200),25, -160,-50,cv::Scalar(255,255,255),1);
	cv::ellipse(pupil,cv::Point(240,60),cv::Size(170,200),25, 125,35,cv::Scalar(255,255,255),1);
	

	cv::imshow("arc", pupil);

	cv::imshow("mask", masked);
	cv::waitKey(0);

}
int main(int argc, char** argv) 
{ 	
	cv::Mat ellipse;
	//eyelid(200,200);

	//points();

	//eyeshape();

	ellipse = makeEllipse(0.4, 0, 0, 125, 125);
	
	cv::namedWindow("ellipse", cv::WINDOW_NORMAL);
	cv::imshow("ellipse", ellipse);
	cv::waitKey(0);

	// cv::Mat arc = cv::Mat::zeros(1000,1000, CV_32F);
	// cv::ellipse(arc,cv::Point(145,255),cv::Size(300,80),3, -140,-40,cv::Scalar(255,255,255),4);	
	// cv::namedWindow("ellipse", cv::WINDOW_NORMAL);
	// cv::imshow("ellipse", arc);
	// cv::waitKey(0);
	return 0; 
} 
