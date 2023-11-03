#pragma once

#include <yarp/os/all.h>
#include <typeinfo>
#include <algorithm>
using namespace yarp::os;

class eyeTracking
{
public:

    enum cam_param_name{w,h,cx,cy,fx,fy};
    std::array<double, 6> cam;
    std::array<double, 5> state;

    std::vector<cv::Mat> template_matrices;
    std::vector<cv::Mat> mexican_templates;

    cv::Mat concat_affines; 


    std::vector<double> scores_vector; 

    cv::Mat eros_filtered, eros_tracked, eros_tracked_64f, eros_resized, cur, current_template, rectangle_eros;
    cv::Point2d initial_position, new_position;
    cv::Point2d new_center; 
    cv::Mat eyelid_shape;

    int blur{11};
    int median_blur_eros{3}; 
    int gaussian_blur_eros{1}; 
    double r, rotation;
    double rot_mat[5][2] = {
        {   1,  0 },
        {  -1,  0 },
        {   0,  1 },
        {   0,  -1},
        // {   1,  1 },
        // {  -1,  -1 },
        // {   -1,  1 },
        // {   1,  -1},
        {   0,  0 }
    };
    double frame_width, frame_height;

    

public:

    void init(double u, double v, double yaw, double pitch, double radius, double ratio, double rot){
        
        state[0]=u; state[1]=v; state[2]=yaw; state[3]=pitch; state[4]=radius;
        
        this->r = ratio;
        this->rotation = rot;

        for (int i = 0; i < sizeof(rot_mat)/sizeof(rot_mat[0]); i++){
		    rot_mat[i][0] = rot*rot_mat[i][0];
		    rot_mat[i][1] = rot*rot_mat[i][1];
	    }

        cur = makeEllipse(state[2], state[3], state[4], state[4]);        
        
        showCurrentEllipse();

    }


    void reset(){

        scores_vector.clear(); 
        mexican_templates.clear(); 

    }


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


    cv::Mat makeEllipse (double theta, double phi, int height, int width){
	
        frame_height = 2*height+1;
        frame_width = 2*width+1;

        cv::Mat ell_filter = cv::Mat::zeros(frame_height, frame_width, CV_32F);
        double x = 0;
        double y = 0;


        for (float t = -M_PI; t < M_PI; t += 2*M_PI/1000){
            // YAW (y) & PITCH (x)
            // x = r*cos(t)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;
            // y = r*sin(t)*cos(phi)-sqrt(1 - pow((x-1),2) - pow(r*sin(t),2))*sin(phi) +1;

            // YAW (Y) & ROLL (z)
            x = (r*cos(theta)*cos(t) + sqrt(1-pow(r,2))*sin(theta))*cos(phi) - r*sin(t)*sin(phi) + 1;
            y = (r*cos(theta)*cos(t) + sqrt(1-pow(r,2))*sin(theta))*sin(phi) + r*sin(t)*cos(phi) + 1;

            // std::cout << "x: " <<  x << "  y: " << y << "  t: " << t << std::endl;

            ell_filter.at<float>(round(y*width),round(x*height)) = 1;

            // if((t > -3*M_PI/4 && t < -M_PI/4) || (t > M_PI/4 && t < 3*M_PI/4)){
			//     ell_filter.at<float>(round(y*width),round(x*height)) = 1;
		    // }
        }

        //resize

        //cv::Rect cur_shape(0, 0, frame_width, std::min(frame_height,frame_height - (frame_height+state[1]- cam[eyeTracking::h])));
        cv::Rect cur_shape(std::max(0.0, (frame_width-cam[eyeTracking::w])/2), std::max(0.0, (frame_height-cam[eyeTracking::h])/2), std::min(cam[eyeTracking::w] - std::max(0.0, state[1]-frame_width/2 -1), frame_width), std::min(cam[eyeTracking::h] - std::max(0.0, state[0]-frame_height/2 -1), frame_height));
        //cv::Mat tempCur = cv::Mat::zeros(std::min(frame_height,frame_height - (frame_height+state[1]- cam[eyeTracking::h])), frame_width , CV_64F);
        cv::Mat tempCur = cv::Mat::zeros(std::min(cam[eyeTracking::w] - std::max(0.0, state[1]-frame_width/2 -1), frame_width), std::min(cam[eyeTracking::h] - std::max(0.0, state[0]-frame_height/2) -1, frame_height), CV_64F);
        ell_filter(cur_shape).copyTo(tempCur);

        // std::cout << ell_filter.size() << std::endl;

        // cv::imshow("filter", ell_filter);

        return tempCur;
    }

    void showCurrentEllipse(){
        current_template = cv::Mat::zeros(cam[eyeTracking::h], cam[eyeTracking::w], CV_64F);
                        
        cv::Rect temp_shape(std::max(0.0, state[1]-frame_width/2), std::max(0.0, state[0]-frame_height/2), std::min(cam[eyeTracking::w] - std::max(0.0, state[1]-frame_width/2 -1), frame_width), std::min(cam[eyeTracking::h] - std::max(0.0, state[0]-frame_height/2 -1), frame_height));

        cur.copyTo(current_template(temp_shape));        
    }


    void createTemplates (int n_templates){


        // draw ellipse accoring to the function omn template matrices black images initialized in the function init

        for (int i =0; i<n_templates; i++){

            cv::Mat template_matrix = makeEllipse(state[2] + rot_mat[i][0], state[3] + rot_mat[i][1], state[4], state[4]); // 8u is gray scale image with numbers from 0 to 255... you could need 32f or 64f in the future

            cv::Mat mexican_template; 
            // mexican blur 
            make_template(template_matrix, mexican_template); 
            mexican_template.convertTo(mexican_template, CV_64F); 

            mexican_templates.push_back(mexican_template); 
        }
        cur = mexican_templates[4];
        
        showCurrentEllipse();

        std::vector<cv::Mat> couple_matrix;
        for (int i=0; i<mexican_templates.size()-1; i++){
                if (i%2==0){
                    cv::Mat mat_left = mexican_templates[i];
                    cv::Mat mat_right = mexican_templates[i+1];
                    cv::Mat current_matrix; 
                    cv::hconcat(mat_left, mat_right, current_matrix);
                    couple_matrix.push_back(current_matrix);
                }

            }

            if (couple_matrix.size() != 0)
                cv::vconcat(couple_matrix, concat_affines); 
        

        couple_matrix.clear(); 
    }

    void setEROS(const cv::Mat &eros)
    {
        //std::cout << eros.size() << std::endl;
        cv::Mat eros_blurred1; 
        cv::medianBlur(eros, eros_blurred1, median_blur_eros);
        cv::GaussianBlur(eros_blurred1, eros_filtered, cv::Size(gaussian_blur_eros, gaussian_blur_eros), 0);


        eros_filtered.convertTo(eros_tracked_64f, CV_64F, 0.003921569); 

        //crop 
        //cv::Rect crop_size = cv::Rect(state[0] - frame_width/2, state[1] - std::min(frame_height,frame_height - (frame_height+state[1]- cam[eyeTracking::h]))/2, frame_width, std::min(frame_height,frame_height - (frame_height+state[1]- cam[eyeTracking::h])));
        cv::Rect crop_size(std::max(0.0, state[1]-frame_width/2), std::max(0.0, state[0]-frame_height/2), std::min(cam[eyeTracking::w] - std::max(0.0, state[1]-frame_width/2 -1), frame_width), std::min(cam[eyeTracking::h] - std::max(0.0, state[0]-frame_height/2 -1), frame_height));
        
        //rectangle_eros = cv::rectangle(rectangle_eros, cv::Point(state[0], state[1]), cv::Point(state[0]+frame_width, state[1]+frame_height), );
        rectangle_eros = cv::Mat::zeros(cam[eyeTracking::h], cam[eyeTracking::w], CV_64F);
        
        cv::rectangle(rectangle_eros, crop_size, cv::Scalar(255,0,0),1);
        //cv::Point(state[0], state[1]), cv::Point(state[0]+frame_width, state[1]+frame_height),
        //rect_shape.copyTo(rectangle_eros(crop_size));
        eros_tracked_64f(crop_size).copyTo(eros_resized);

        cv::namedWindow("EROS", cv::WINDOW_NORMAL);
        cv::imshow("EROS", eros_resized);




        cv::Mat current_matrix;
        cv::Mat concat_eros;
        cv::Mat con_eros_crop;

        eros(crop_size).copyTo(con_eros_crop);
        con_eros_crop.convertTo(con_eros_crop, CV_64F, 0.003921569); 

        cv::hconcat(con_eros_crop, con_eros_crop, current_matrix);

        cv::vconcat(current_matrix, current_matrix, concat_eros); 

        cv::namedWindow("templates", cv::WINDOW_NORMAL);
        cv::imshow("templates", concat_affines + concat_eros);




        
    }

    double similarity_score(const cv::Mat &observation, const cv::Mat &expectation) {
        static cv::Mat muld;
        muld = expectation.mul(observation);
        return cv::sum(cv::sum(muld))[0];
    }

    void performComparisons(){
        for (int t = 0; t < mexican_templates.size(); t++) {
            double score_template = similarity_score(eros_resized, mexican_templates[t]);
            scores_vector.push_back(score_template);
        }
    }

    void updateState(){

        double no_motion = scores_vector[4];
        int best_score_index = std::max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        double best_score = *max_element(scores_vector.begin(), scores_vector.end());
        // yInfo() << scores_vector;
        // yInfo() << "highest score =" << best_score_index << best_score;
        if(no_motion!=0){
            if (best_score_index == 0)
                state[2] += rotation;
            else if (best_score_index == 1)
                state[2] -= rotation;
            else if (best_score_index == 2)
                state[3] += rotation;
            else if (best_score_index == 3)
                state[3] -= rotation;
        }

    }

    void updateStateAll(){

        // int best_score_index = max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        // double best_score = *max_element(scores_vector.begin(), scores_vector.end());
        // yInfo() << scores_vector;
        // yInfo() << "highest score =" << best_score_index << best_score;
        double no_motion = scores_vector[4];

        if(scores_vector[0] > no_motion) state[2] += rotation;
        if(scores_vector[1] > no_motion) state[2] -= rotation;
        if(scores_vector[2] > no_motion) state[3] += rotation;
        if(scores_vector[3] > no_motion) state[3] -= rotation;

    }

    void updateStateCustom4(){
        double no_motion = scores_vector[4];
        int best_score_index = std::max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        int best_score_index2 = std::max_element(scores_vector.begin(), scores_vector.end()-1) - scores_vector.begin();

        if(no_motion < 6 && no_motion!=0){
            if (best_score_index2 == 0)
                state[2] += rotation;
            else if (best_score_index2 == 1)
                state[2] -= rotation;
            else if (best_score_index2 == 2)
                state[3] += rotation;
            else if (best_score_index2 == 3)
                state[3] -= rotation;
        }
        
    }


    void updateStateCustomBest(){
        double no_motion = scores_vector[8];
        int best_score_index = std::max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        int best_score_index2 = std::max_element(scores_vector.begin(), scores_vector.end()-1) - scores_vector.begin();

        if(no_motion < 10 && no_motion!=0){
            if (best_score_index2 == 0)
                state[2] += rotation;
            else if (best_score_index2 == 1)
                state[2] -= rotation;
            else if (best_score_index2 == 2)
                state[3] += rotation;
            else if (best_score_index2 == 3)
                state[3] -= rotation;
            else if (best_score_index2 == 4){
                state[2] += rotation;
                state[3] += rotation;
            }
                
            else if (best_score_index2 == 5){
                state[2] -= rotation;
                state[3] +- rotation;
            }
            else if (best_score_index2 == 6){
                state[2] -= rotation;
                state[3] += rotation;
            }
            else if (best_score_index2 == 7){
                state[2] += rotation;
                state[3] -= rotation;
            }

        
        }

    }


    void updateStateCustom(){
        double no_motion = scores_vector[4];
        int best_score_index = std::max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        int best_score_index2 = std::max_element(scores_vector.begin(), scores_vector.end()-1) - scores_vector.begin();

        if(no_motion < 10 && no_motion!=0){
            if (scores_vector[0] > 0.3*no_motion)
                state[2] += rotation;
            if (scores_vector[1] > 0.3*no_motion)
                state[2] -= rotation;
            if (scores_vector[2] > 0.3*no_motion)
                state[3] += rotation;
            if (scores_vector[3] > 0.3*no_motion)
                state[3] -= rotation;

        
        }

    }


   void eyelid(){
        eyelid_shape = cv::Mat::zeros(cam[eyeTracking::h], cam[eyeTracking::w], CV_64F);

        double a = 2.01757968*pow(10,-7);
        double b = -1.34554144*pow(10,-4);
        double c = 3.60365584*pow(10,-2);
        double d = -4.73514210*pow(10,0);
        double e = 3.79484549*pow(10,2);

        for(int x = 82; x <300; x++){
            int y = a*pow(x,4)+b*pow(x,3)+c*pow(x,2)+d*x+e;
            eyelid_shape.at<double>(round(y), x) = 1;
        }


        cv::namedWindow("eyelid", cv::WINDOW_NORMAL);
        cv::imshow("eyelid", eyelid_shape);

    }
};