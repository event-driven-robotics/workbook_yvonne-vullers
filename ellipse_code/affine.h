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

    std::array<double, 4> updates;

    std::vector<cv::Mat> mexican_templates;
    cv::Mat concat_affines; 
    std::vector<double> scores_vector; 

    cv::Mat eros_filtered, eros_tracked_64f, eros_resized, cur, current_template, rectangle_eros, vis_ellipse;
    cv::Mat centers;

    int blur{15};
    int median_blur_eros{3}; 
    int gaussian_blur_eros{1}; // CHANGE THIS
    double r, rotation;
    double rot_mat[5][2] = {
        {   1,  0 },
        {  -1,  0 },
        {   0,  1 },
        {   0,  -1},
        {   0,  0 }
    };
    double frame_width, frame_height;
    float centre_u, centre_v;
    cv::Mat centre_cut, centre_full;
    cv::Rect filter_crop, crop_shape;
    int filter_shape_x,filter_shape_y;
    bool speed;

    double center_x_small, center_y_small, center_x, center_y;
    double xmax, xmin, ymax, ymin;
    double scale;
    

public:

    void init(double u, double v, double yaw, double pitch, double radius, double ratio, double rot, bool fast){
        
        state[0]=u; state[1]=v; state[2]=yaw; state[3]=pitch; state[4]=radius;
        
        this->r = ratio;
        this->rotation = rot;
        scale = sqrt(1-pow(r,2));

        speed = fast;

        for (int i = 0; i < sizeof(rot_mat)/sizeof(rot_mat[0]); i++){
		    rot_mat[i][0] = (rot/radius)*rot_mat[i][0];
		    rot_mat[i][1] = (rot/radius)*rot_mat[i][1];
	    }

        // only works when 1 size is used
        frame_height = 2*radius+1;
        frame_width = 2*radius+1;

        filter_crop = cv::Rect(std::max(0.0, (frame_width-cam[eyeTracking::w])/2), std::max(0.0, (frame_height-cam[eyeTracking::h])/2), std::min(cam[eyeTracking::w] - std::max(0.0, state[1]-frame_width/2 -1), frame_width), std::min(cam[eyeTracking::h] - std::max(0.0, state[0]-frame_height/2 -1), frame_height));
        filter_shape_x = std::min(cam[eyeTracking::w] - std::max(0.0, state[1]-frame_width/2 -1), frame_width);
        filter_shape_y =  std::min(cam[eyeTracking::h] - std::max(0.0, state[0]-frame_height/2) -1, frame_height);

        crop_shape = cv::Rect(std::max(0.0, state[1]-frame_width/2), std::max(0.0, state[0]-frame_height/2), std::min(cam[eyeTracking::w] - std::max(0.0, state[1]-frame_width/2 -1), frame_width), std::min(cam[eyeTracking::h] - std::max(0.0, state[0]-frame_height/2 -1), frame_height));



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
	
        // frame_height = 2*height+1;
        // frame_width = 2*width+1;

        cv::Mat ell_filter = cv::Mat::zeros(frame_height, frame_width, CV_32F);
        double x = 0;
        double y = 0;

        
        float cos_theta = cos(theta);
        float sin_theta = sin(theta);
        float root = sqrt(1-pow(r,2));
        float sin_phi = sin(phi);
        float rcos_phi = r*cos(phi);
        ymin = 2;
        ymax = 0;

        for (float t = -M_PI; t < M_PI; t += 2*M_PI/1000){
            // YAW (y) & PITCH (x)
            // if((t > -M_PI/4 && t < M_PI/4)||(t > 3*M_PI/4 ||t < -3*M_PI/4)){
            x = r*cos(t)*cos_theta + root*sin_theta + 1;
            y = (r*cos(t)*sin_theta-root*cos_theta)*sin_phi + rcos_phi*sin(t) + 1;

            // YAW (Y) & ROLL (z)
            // x = (r*cos(theta)*cos(t) + sqrt(1-pow(r,2))*sin(theta))*cos(phi) - r*sin(t)*sin(phi) + 1;
            // y = (r*cos(theta)*cos(t) + sqrt(1-pow(r,2))*sin(theta))*sin(phi) + r*sin(t)*cos(phi) + 1;

            // std::cout << "x: " <<  x << "  y: " << y << "  t: " << t << std::endl;

            // ell_filter.at<float>(round(y*width),round(x*height)) = 1;


            if (y > ymax) ymax = y;
            if (y < ymin) ymin = y;

            

            if((t > -M_PI/4 && t < M_PI/4)||(t > 3*M_PI/4 ||t < -3*M_PI/4)){        
                ell_filter.at<float>(round(y*width),round(x*height)) = 1;
                
            }
            // }
        }
        

        cv::Mat tempCur = cv::Mat::zeros(filter_shape_x, filter_shape_y, CV_32F);

        ell_filter(filter_crop).copyTo(tempCur);

        if (speed == false){
            vis_ellipse = cv::Mat::zeros(frame_height, frame_width, CV_32F);

            centre_u = round(((sqrt(1))*sin(theta)*cos(phi)+1)*width);
            centre_v = round(((sqrt(1))*sin(theta)*sin(phi)+1)*height);

            vis_ellipse.at<float>(centre_v,centre_u) = 1;
            vis_ellipse.at<float>(centre_v+1,centre_u) = 1;
            vis_ellipse.at<float>(centre_v-1,centre_u) = 1;
            vis_ellipse.at<float>(centre_v,centre_u+1) = 1;
            vis_ellipse.at<float>(centre_v,centre_u-1) = 1;

            // std::cout << centre_v << ", " << centre_u << std::endl;
            // std::cout << state[2] << ", " << state[3] << std::endl;
            centre_cut = cv::Mat::zeros(filter_shape_x, filter_shape_y, CV_64F);
            vis_ellipse(filter_crop).copyTo(centre_cut);

        }

        return tempCur;
    }

    void showCurrentEllipse(){
        current_template = cv::Mat::zeros(cam[eyeTracking::h], cam[eyeTracking::w], CV_64F);
                        
        cur.copyTo(current_template(crop_shape));        
        
        if (speed == false){
            centre_full = cv::Mat::zeros(cam[eyeTracking::h], cam[eyeTracking::w], CV_64F);
            centre_cut.copyTo(centre_full(crop_shape));  
        }    
    }


    void createTemplates (int n_templates){
        float update_theta, update_phi;
        cv::Mat template_matrix_last;

        for (int i =0; i<n_templates-1; i++){
            update_theta = 0;
            update_phi = 0;
            

            if (i == 0){
                template_matrix_last = makeEllipse(state[2], state[3], state[4], state[4]);
                eyeCenter();

            }


            if (rot_mat[i][0] == 0 && rot_mat[i][1] != 0){
                update_phi = asin(std::min(1.0, std::max(-1.0,-(center_y_small+rot_mat[i][1]-1)/(scale*cos(state[2]))))) - state[3];
                updates[i] = update_phi;
                // np.arcsin(-(y_center2+pixel_shift)/(scale*np.cos(theta))) - phi
            }
            else if (rot_mat[i][1] == 0 && rot_mat[i][0] != 0){
                update_theta = asin(std::min(1.0, std::max(-1.0, (center_x_small+rot_mat[i][0]-1)/scale))) - state[2];
                updates[i] = update_theta;
                // np.arcsin((x_center2+pixel_shift)/scale) - theta
            }
            
            cv::Mat template_matrix = makeEllipse(state[2] + update_theta, state[3] + update_phi, state[4], state[4]); // 8u is gray scale image with numbers from 0 to 255... you could need 32f or 64f in the future

            cv::Mat mexican_template; 
            // mexican blur 
            make_template(template_matrix, mexican_template); 
            mexican_template.convertTo(mexican_template, CV_64F); 

            mexican_templates.push_back(mexican_template); 
        }

        cv::Mat mexican_template; 
        make_template(template_matrix_last, mexican_template); 
        mexican_template.convertTo(mexican_template, CV_64F); 

        mexican_templates.push_back(mexican_template); 
        cur = mexican_template;
        
        showCurrentEllipse();

        if (speed == false){
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
    }

    void setEROS(const cv::Mat &eros)
    {
        // cv::Mat eros_blurred1; 
        // cv::medianBlur(eros, eros_blurred1, median_blur_eros);
        // cv::GaussianBlur(eros_blurred1, eros_filtered, cv::Size(gaussian_blur_eros, gaussian_blur_eros), 0);

        cv::medianBlur(eros, eros_filtered, median_blur_eros);
        eros_filtered.convertTo(eros_tracked_64f, CV_64F, 0.003921569); 

        eros_tracked_64f(crop_shape).copyTo(eros_resized);

        if (speed == false){
            rectangle_eros = cv::Mat::zeros(cam[eyeTracking::h], cam[eyeTracking::w], CV_64F);
            cv::rectangle(rectangle_eros, crop_shape, cv::Scalar(255,0,0),1);
            //rect_shape.copyTo(rectangle_eros(crop_size));

            cv::Mat current_matrix;
            cv::Mat concat_eros;
            cv::Mat con_eros_crop;

            eros(crop_shape).copyTo(con_eros_crop);
            con_eros_crop.convertTo(con_eros_crop, CV_64F, 0.003921569); 

            cv::hconcat(con_eros_crop, con_eros_crop, current_matrix);

            cv::vconcat(current_matrix, current_matrix, concat_eros); 

            cv::namedWindow("templates", cv::WINDOW_NORMAL);
            cv::imshow("templates", concat_affines + concat_eros);

            cv::namedWindow("EROS", cv::WINDOW_NORMAL);
            cv::imshow("EROS", eros_tracked_64f);
            
        }
        
        
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

    // void updateState(){

    //     double no_motion = scores_vector[4];
    //     int best_score_index = std::max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
    //     double best_score = *max_element(scores_vector.begin(), scores_vector.end());

    //     if(no_motion!=0){
    //         if (best_score_index == 0)
    //             state[2] += rotation;
    //         else if (best_score_index == 1)
    //             state[2] -= rotation;
    //         else if (best_score_index == 2)
    //             state[3] += rotation;
    //         else if (best_score_index == 3)
    //             state[3] -= rotation;
    //     }

    // }

    void updateState(){

        double no_motion = scores_vector[4];
        int best_score_index = std::max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        double best_score = *max_element(scores_vector.begin(), scores_vector.end());

        if(no_motion!=0){
            if (best_score_index == 0)
                state[2] += updates[0];
            else if (best_score_index == 1)
                state[2] += updates[1];
            else if (best_score_index == 2)
                state[3] += updates[2];
            else if (best_score_index == 3)
                state[3] += updates[3];
        }

        // std::cout << state[2]<< ", " << state[3] << std::endl;

    }

    void updateStateAll(){
        double no_motion = scores_vector[4];

        if(scores_vector[0] > no_motion) state[2] += rotation;
        if(scores_vector[1] > no_motion) state[2] -= rotation;
        if(scores_vector[2] > no_motion) state[3] += rotation;
        if(scores_vector[3] > no_motion) state[3] -= rotation;

    }


    void eyeCenter(){
        double theta = state[2];
        double phi = state[3];
        double radius = state[4];

        center_x_small = sqrt(1-pow(r,2))*sin(theta) + 1;
        center_y_small = sqrt(1-pow(r,2))*(-sin(phi)*cos(theta)) + 1;
        center_x = round(center_x_small*state[4]) + state[1] - state [4];
        center_y = round(center_y_small*state[4]) + state[0] - state[4];
        // center_x = round(((xmax+xmin)/2)*radius) + state[1] - radius;
        // center_y = round(((ymax+ymin)/2)*radius) + state[0] - radius;

        xmax = r*cos(0)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;
        xmin = r*cos(M_PI)*cos(theta)+sqrt(1-pow(r,2))*sin(theta) + 1;

        xmax = round(xmax*radius)+state[1] - radius;
        xmin = round(xmin*radius)+state[1] - radius;
        ymax = round(ymax*radius)+state[0] - radius;
        ymin = round(ymin*radius)+state[0] - radius;

        if (speed == false){
            centers = cv::Mat::zeros(260, 346, CV_32F);

            centers.at<float>(round(center_y),round(center_x)) = 1;
            centers.at<float>(round(center_y+1),round(center_x)) = 1;
            centers.at<float>(round(center_y-1),round(center_x)) = 1;
            centers.at<float>(round(center_y),round(center_x+1)) = 1;
            centers.at<float>(round(center_y),round(center_x-1)) = 1;

            centers.convertTo(centers, CV_64F);
        }

    }
};