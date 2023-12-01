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

float center_x, center_y;
int blur{11};

void make_template(const cv::Mat &input, cv::Mat &output) {
	
	static cv::Mat pos_hat, neg_hat;
	static cv::Size pblur(blur, blur);
	static cv::Size nblur(2*blur-1, 2*blur-1);
	static double minval, maxval;

	cv::GaussianBlur(input, pos_hat, pblur, 0);
	cv::GaussianBlur(input, neg_hat, nblur, 0);

	output = pos_hat - neg_hat;

	cv::minMaxLoc(output, &minval, &maxval);
	double scale_factor = 1.0 / (2 * std::max(fabs(minval), fabs(maxval)));
	output *= scale_factor;

}

cv::Mat makeEllipse (float r, double theta, double phi, int height, int width){
	cv::Mat ell_filter = cv::Mat::zeros(2*height+1, 2*width+1, CV_32F);	// CHECK +1
	double x = 0;
	double y = 0;

	double xmaxy, xminy;
	double xmax = 0;
	double ymax = 0;
	double xmin = 2;
	double ymin = 2;


	for (float t = -M_PI; t < M_PI; t += 2*M_PI/1000){
		// YAW (y) & PITCH (x)
		// x = r*cos(t)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;
		// y = r*sin(t)*cos(phi)-sqrt(1 - pow((x-1),2) - pow(r*sin(t),2))*sin(phi) +1;

		x = r*cos(t)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;
    	y = (r*sin(theta)*cos(t)-sqrt(1-pow(r,2))*cos(theta))*sin(phi) + r*sin(t)*cos(phi) + 1;


		//std::cout<<x << ", " << y << std::endl;
		// YAW (Y) & ROLL (z)
		// x = (r*cos(theta)*cos(t) + sqrt(1-pow(r,2))*sin(theta))*cos(phi) - r*sin(t)*sin(phi) + 1;
		// y = (r*cos(theta)*cos(t) + sqrt(1-pow(r,2))*sin(theta))*sin(phi) + r*sin(t)*cos(phi) + 1;

		// std::cout << "x: " <<  x << "  y: " << y << "  t: " << t << std::endl;

		// if((t > -M_PI/4 && t < M_PI/4) || (t > 3*M_PI/4 || t < -3*M_PI/4)){
			ell_filter.at<float>(round(y*width),round(x*height)) = 1;
		// }

		// if (x > xmax) xmax = x;
		// if (y > ymax) ymax = y;
		// if (x < xmin) xmin = x;
		// if (y < ymin) ymin = y;

	}
	// center of pupil?
	center_x = (sin(theta) + 1)*height;
	center_y = (-sin(phi)*cos(theta)+1)*height;

	std::cout << center_x << ", " << center_y << std::endl; 

	// center of ellipse
	// xmax = r*cos(0)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;
	// xmin = r*cos(M_PI)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;
	// xmaxy = r*cos(M_PI/2)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;
	// xminy = r*cos(3*M_PI/2)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;
	// ymax = r*sin(M_PI/2)*cos(phi)-sqrt(1 - pow((xmaxy-1),2) - pow(r*sin(M_PI/2),2))*sin(phi) +1;
	// ymin = r*sin(3*M_PI/2)*cos(phi)-sqrt(1 - pow((xminy-1),2) - pow(r*sin(3*M_PI/2),2))*sin(phi) +1;

	// center_x = (xmax+xmin)/2;
	// center_y = (ymax+ymin)/2;

	//std::cout << round(center_x*width) + 220 - width << ", " << round(center_y*width) + 185 - width << std::endl;
	//std::cout << ell_filter.size() << std::endl;

	ell_filter.at<float>(round(center_y),round(center_x)) = 1;
	ell_filter.at<float>(round(center_y+1),round(center_x)) = 1;
	ell_filter.at<float>(round(center_y-1),round(center_x)) = 1;
	ell_filter.at<float>(round(center_y),round(center_x+1)) = 1;
	ell_filter.at<float>(round(center_y),round(center_x-1)) = 1;


	return ell_filter;
}

cv::Point maxlocs (cv::Mat binaryImage){
	cv::Mat locations;   // output, locations of non-zero pixels
	cv::findNonZero(binaryImage, locations);

	double minVal, maxVal;
	cv::minMaxLoc(locations, &minVal, &maxVal);

	cv::Point pnt = cv::Point(0,0);
	
	std::cout << minVal << std::endl;
	return pnt;
}

void points(){
	cv::Mat arc = cv::Mat::zeros(400,400, CV_32F);

	cv::ellipse(arc,cv::Point(185,90),cv::Size(200,80),10, 40,110,cv::Scalar(255,255,255),2);
	for(int i = 0; i <400; i++){
		for(int j = 0; j < 400; j++){
			if(arc.at<float>(i,j) == 255){
				std::cout << j << ", " ; //<< 399-i << std::endl;
			}
		}
	}

	// cv::namedWindow("circle", cv::WINDOW_NORMAL);
	// cv::imshow("circle", arc);
	// cv::waitKey(0);
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
	cv::Mat mexican_template, colored;
	cv::Mat one = cv::Mat::ones(2*98+1,2*98+1, CV_32F);

	//eyelid(200,200);

	// points();

	//eyeshape();

	ellipse = makeEllipse(0.5, M_PI/4, 0, 98, 98); 

	make_template(ellipse, mexican_template);
	mexican_template.at<float>(98,98) = 1;

	//std::cout << mexican_template << std::endl;
	mexican_template = mexican_template + one;
	mexican_template.convertTo(mexican_template, CV_8UC1, 255/2); 

	cv::applyColorMap(mexican_template, colored, cv::COLORMAP_JET);
	
	// double theta = M_PI/4;
	// double phi = M_PI/4;

	// std::cout << -1*(sin(theta)+1)*98 << ", " << 1*(sin(phi)+1)*98 << std::endl;

	// cv::arrowedLine(ellipse, cv::Point(98,98), cv::Point((1*sin(theta)+1)*98, (-1*sin(phi)+1)*98),cv::Scalar(255,255,255) );
	cv::namedWindow("ellipse", cv::WINDOW_NORMAL);
	cv::imshow("ellipse", mexican_template);
	cv::waitKey(0);
	

	// cv::Mat arc = cv::Mat::zeros(1000,1000, CV_32F);
	// cv::ellipse(arc,cv::Point(145,255),cv::Size(300,80),3, -140,-40,cv::Scalar(255,255,255),4);	
	// cv::namedWindow("ellipse", cv::WINDOW_NORMAL);
	// cv::imshow("ellipse", arc);
	// cv::waitKey(0);
	return 0; 
} 
