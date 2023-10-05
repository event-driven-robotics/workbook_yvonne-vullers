#pragma once

#include <yarp/os/all.h>

using namespace yarp::os;

class affineTransforms
{
public:

    enum cam_param_name{w,h,cx,cy,fx,fy};

    typedef struct affine_struct
    {
        cv::Mat A;
        cv::Mat warped_img;
        double score;
        cv::Mat rmp;
        cv::Mat rmsp;
        int axis{0};
        double delta{0.0};

    } affine_bundle;

    std::array<affine_bundle, 9> affine_info;
    std::vector<cv::Mat> affines_vector;
    std::vector<double> scores_vector;
    std::array<double, 4> state;

    cv::Rect2d roi_around_shape, square;
    cv::Mat initial_template, roi_template, roi_template_64f, roi_resized, mexican_template, mexican_resized, mexican_template_64f;
    cv::Mat eros_filtered, eros_tracked, eros_tracked_64f, eros_resized;
    cv::Mat rot_scaled_tr_template;
    double translation, angle, pscale, nscale;
    cv::Point2d initial_position, new_position;
    cv::Point2d new_center; 
    int blur{11};
    int gaussian_blur_eros{1}; 
    double template_scale{0.4}; //0.4, 0.33
    double sc_factor{0.8}; 
    int shape_blur_val{7};
    int buffer_width; 
    int canny1{100}; 
    int canny2{300};
    int sobel_value{3};
    double median_blur_eros{3}; 
    bool median_blur_active{true}; 
    bool dynamic_scale{false};

    cv::Size proc_size{cv::Size(80, 80)};
    cv::Rect proc_roi, o_proc_roi; 
    cv::Mat prmx, prmy, nrmx, nrmy;
    
    std::array<double, 6> cam;

public:

    void init(double trans, double rot, double ps, double ns){
        
        roi_resized = cv::Mat(proc_size, CV_64F);
        eros_resized = cv::Mat(proc_size, CV_64F);
        mexican_resized = cv::Mat(proc_size, CV_64F);
        o_proc_roi = proc_roi = cv::Rect(cv::Point(0, 0), proc_size);

        for(auto &affine : affine_info) {
            affine.A = cv::Mat::zeros(2,3, CV_64F);
            affine.warped_img = cv::Mat::zeros(proc_size, CV_64F);
        }

        prmx = cv::Mat::zeros(proc_size, CV_32F);
        prmy = cv::Mat::zeros(proc_size, CV_32F);
        nrmx = cv::Mat::zeros(proc_size, CV_32F);
        nrmy = cv::Mat::zeros(proc_size, CV_32F);

        this->translation = trans;
        this->angle = rot;
        this->pscale = ps;
        this->nscale = ns;
    }

    void initState(){
        state[0]=0; state[1]=0; state[2]=0; state[3]=1;
    }

    void createAffines(double translation, cv::Point2f center, double angle, double pscale, double nscale){

        affine_info[0].A.at<double>(0,0) = 1;
        affine_info[0].A.at<double>(0,2) = translation;
        affine_info[0].A.at<double>(1,1) = 1;
        affine_info[0].A.at<double>(1,2) = 0;

        affine_info[1].A.at<double>(0,0) = 1;
        affine_info[1].A.at<double>(0,2) = -translation;
        affine_info[1].A.at<double>(1,1) = 1;
        affine_info[1].A.at<double>(1,2) = 0;

        affine_info[2].A.at<double>(0,0) = 1;
        affine_info[2].A.at<double>(0,2) = 0;
        affine_info[2].A.at<double>(1,1) = 1;
        affine_info[2].A.at<double>(1,2) = translation;

        affine_info[3].A.at<double>(0,0) = 1;
        affine_info[3].A.at<double>(0,2) = 0;
        affine_info[3].A.at<double>(1,1) = 1;
        affine_info[3].A.at<double>(1,2) = -translation;

        affine_info[4].A = cv::getRotationMatrix2D(center, angle, 1);
        affine_info[5].A = cv::getRotationMatrix2D(center, -angle, 1);

        affine_info[6].A = cv::getRotationMatrix2D(center, 0, pscale);
        affine_info[7].A = cv::getRotationMatrix2D(center, 0, nscale);

        affine_info[8].A.at<double>(0,0) = 1;
        affine_info[8].A.at<double>(1,1) = 1;
    }

    void updateAffines(){

        new_center = cv::Point2d(roi_around_shape.width/2, roi_around_shape.height/2);

        affine_info[4].A = cv::getRotationMatrix2D(new_center, angle, 1);
        affine_info[5].A = cv::getRotationMatrix2D(new_center, -angle, 1);

        affine_info[6].A = cv::getRotationMatrix2D(new_center, 0, pscale);
        affine_info[7].A = cv::getRotationMatrix2D(new_center, 0, nscale);
    }

    void setROI(int buffer = 10){

        roi_resized = 0; 
        buffer_width = buffer; 
        static cv::Rect2d full_roi = cv::Rect2d(cv::Point(0, 0), rot_scaled_tr_template.size());

        roi_around_shape = cv::boundingRect(rot_scaled_tr_template);

        if(roi_around_shape.width == 0) {
            roi_around_shape = full_roi;
            return;
        }
        roi_around_shape.x -= buffer;
        roi_around_shape.y -= buffer;
        roi_around_shape.width += buffer * 2;
        roi_around_shape.height += buffer * 2;

        roi_around_shape = roi_around_shape & full_roi;

        roi_template = rot_scaled_tr_template(roi_around_shape);
        roi_template.convertTo(roi_template_64f, CV_64F);  // is 6 type CV_64FC1

        cv::resize(roi_template_64f, roi_resized(proc_roi), proc_roi.size(), 0, 0, cv::INTER_LINEAR);
        make_template(roi_resized, mexican_resized);
        // cv::Mat output2; 
        // make_template2(roi_resized, mexican_resized); 

        imshow("template", mexican_resized+0.5); 
    }

    cv::Mat updateTrMat(double translation_x, double translation_y){

        // the state is an affine
        cv::Mat trMat = cv::Mat::zeros(2,3, CV_64F);

        trMat.at<double>(0,0) = 1; trMat.at<double>(0,1) = 0; trMat.at<double>(0,2) = translation_x;
        trMat.at<double>(1,0) = 0; trMat.at<double>(1,1) = 1; trMat.at<double>(1,2) = translation_y;

        return trMat;
    }

    void createStaticTemplate(const cv::Size &res, double scaling_percentage){

        initial_template = cv::Mat::zeros(res.height, res.width, CV_8UC1);
        double square_width = res.height*(scaling_percentage/100);
        double square_height = res.height*(scaling_percentage/100);
        square = cv::Rect2d((res.width-square_width)/2,(res.height-square_height)/2, square_width, square_height);
        cv::rectangle(initial_template, square, cv::Scalar(255),1,8,0);

        initial_position.x = square.x + square.width/2;
        initial_position.y = square.y + square.height/2;

    }

    void squareTemplate (const cv::Size &res){
        initial_template = cv::Mat::zeros(res.height, res.width, CV_8UC1);
        square = cv::Rect(306,263, 87, 93);
        cv::rectangle(initial_template, square, cv::Scalar(255),1,8,0);

        initial_position.x = square.x + square.width/2;
        initial_position.y = square.y + square.height/2;
    }

    void starTemplate(const cv::Size &res){

        initial_template = cv::Mat::zeros(res.height, res.width, CV_8UC1);

        cv::line(initial_template, cv::Point(327,172), cv::Point(336,204), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(336,204), cv::Point(367,208), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(367,208), cv::Point(341,226), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(341,226), cv::Point(348,258), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(348,258), cv::Point(324,237), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(324,237), cv::Point(296,256), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(296,256), cv::Point(308,225), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(308,225), cv::Point(283,203), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(283,203), cv::Point(316,202), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(316,202), cv::Point(327,172), cv::Scalar(255),1,8,0);

        initial_position.x = 325;
        initial_position.y = 219;
    }

    void triangleTemplate(const cv::Size &res){

        initial_template = cv::Mat::zeros(res.height, res.width, CV_8UC1);

        cv::line(initial_template, cv::Point(320,190), cv::Point(270,290), cv::Scalar(255),2,8,0);
        cv::line(initial_template, cv::Point(320,190), cv::Point(370,290), cv::Scalar(255),2,8,0);
        cv::line(initial_template, cv::Point(270,290), cv::Point(370,290), cv::Scalar(255),2,8,0);

        initial_position.x = 320;
        initial_position.y = 240;
    }

    void loadTemplate(const cv::Size &res, std::string filename){

        cv::Mat shape_image = cv::imread(filename, 0);
        // shape_image.convertTo(shape_image, CV_32F);

        cv::resize(shape_image, shape_image, cv::Size(template_scale*shape_image.cols, template_scale*shape_image.rows), 0, 0,  cv::INTER_LINEAR); 

        // yInfo()<<template_scale*shape_image.cols<< template_scale*shape_image.rows;

        static cv::Mat shape_blur;
        cv::GaussianBlur(shape_image, shape_blur, cv::Size(shape_blur_val,shape_blur_val),0,0);

        // cv::Mat sobelxy;
        // cv::Sobel(shape_blur, sobelxy, CV_64F, 1, 1, 5);

        static cv::Mat edges;
        cv::Canny(shape_blur, edges, canny1, canny2, sobel_value, false);

        initial_template = cv::Mat::zeros(res.height, res.width, CV_8UC1);

        initial_position.x = res.width/2;
        initial_position.y = res.height/2;

        cv::Rect mask = cv::Rect(initial_position.x - shape_image.cols/2, initial_position.y - shape_image.rows/2, shape_image.cols, shape_image.rows); 

        edges.copyTo(initial_template(mask)); 

    }

    void createDynamicTemplate(){

        // cv::Mat rot_scaled_template;
        // rot_scaled_tr_template = 0; 

        cv::Mat rotMatfunc = getRotationMatrix2D(initial_position, state[2], state[3]);
        // cv::warpAffine(initial_template, rot_scaled_template, rotMatfunc, rot_scaled_template.size());
        // cv::Mat trMat =  updateTrMat(state[0], state[1]);
        rotMatfunc.at<double>(0,2) += (state[0]);
        rotMatfunc.at<double>(1,2) += (state[1]);
        cv::warpAffine(initial_template, rot_scaled_tr_template, rotMatfunc, initial_template.size());
        // cv::warpAffine(rot_scaled_template, rot_scaled_tr_template, rotMatfunc, rot_scaled_template.size());
        new_position = cv::Point2d(initial_position.x+state[0],initial_position.y+state[1]);

    }

    void make_template(const cv::Mat &input, cv::Mat &output) {
        
        static cv::Mat canny_img, f, pos_hat, neg_hat;
        static cv::Size pblur(blur, blur);
        static cv::Size nblur(2*blur-1, 2*blur-1);
        static double minval, maxval;

        cv::GaussianBlur(input, pos_hat, pblur, 0);
        cv::GaussianBlur(input, neg_hat, nblur, 0);
        // cv::GaussianBlur(roi_template_64f, pos_hat, pblur, 0);
        // cv::GaussianBlur(roi_template_64f, neg_hat, nblur, 0);
        output = pos_hat - neg_hat;

        cv::minMaxLoc(output, &minval, &maxval);
        double scale_factor = 1.0 / (2 * std::max(fabs(minval), fabs(maxval)));
        output *= scale_factor;
    }

    void make_template2(const cv::Mat &input, cv::Mat &output) {
        static cv::Mat f, pos_hat, neg_hat;
        static cv::Size pblur(blur, blur);
        static cv::Size nblur(2*blur-1, 2*blur-1);
        static double minval, maxval;
        cv::Mat pos_hat_resized = cv::Mat::zeros(input.rows, input.cols, CV_64F); 
        cv::Size pos_size(input.size()-nblur); 

        cv::GaussianBlur(input, pos_hat, pblur, 0);

        cv::resize(pos_hat, pos_hat_resized(cv::Rect({blur, blur}, pos_size)), pos_size, 0, 0, cv::INTER_LINEAR);


        // cv::GaussianBlur(input, neg_hat, nblur, 0);
        // cv::GaussianBlur(roi_template_64f, pos_hat, pblur, 0);
        // cv::GaussianBlur(roi_template_64f, neg_hat, nblur, 0);
        output = pos_hat - pos_hat_resized;

        cv::minMaxLoc(output, &minval, &maxval);
        double scale_factor = 1.0 / (2 * std::max(fabs(minval), fabs(maxval)));
        output *= scale_factor;

    }

    void createWarpings(){

        mexican_template.convertTo(mexican_template_64f, CV_64F);  // is 6 type CV_64FC1

        // pscale = double(roi_template.rows + 0.3)/double(roi_template.rows);
        // nscale = double(roi_template.rows - 0.3)/double(roi_template.rows);

        for (int affine = 0; affine < affine_info.size(); affine++) {
            cv::warpAffine(mexican_template_64f, affine_info[affine].warped_img, affine_info[affine].A, mexican_template_64f.size());

            // cv::imshow("affine remap" + std::to_string(affine), affine_info[affine].warped_img); 
        }
    }

    void createMapWarpings(){

        std::vector<cv::Mat> affine_matrices;
        std::vector<cv::Mat> couple_matrix;

        cv::Mat concat_affines; 
        mexican_resized.convertTo(mexican_template_64f, CV_64F);  // is 6 type CV_64FC1

        // cv::Rect selected_roi(3, 3, mexican_template_64f.cols, mexican_template_64f.rows); 

        if (dynamic_scale){
            pscale = double(roi_template.rows + sc_factor)/double(roi_template.rows);
            nscale = double(roi_template.rows - sc_factor)/double(roi_template.rows);
            create_map_scale(pscale, 6);
            create_map_scale(nscale, 7);
        }            
        
        // yInfo()<<pscale<<nscale;

        for (int affine = 0; affine < affine_info.size()-1; affine++) {
            cv::remap(mexican_template_64f, affine_info[affine].warped_img, affine_info[affine].rmp, affine_info[affine].rmsp, cv::INTER_LINEAR);
            // cv::Mat new_matrix_with_borders = 255*cv::Mat::ones(mexican_template_64f.rows+6, mexican_template_64f.cols+6, CV_64F);

            // affine_info[affine].warped_img.copyTo(new_matrix_with_borders(selected_roi));
            affine_matrices.push_back(affine_info[affine].warped_img);

            // cv::imshow("affine remap" + std::to_string(affine), affine_info[affine].warped_img); 
        }

        affine_info[8].warped_img = mexican_template_64f; 
        // yInfo()<<first_time<<"and"<<affine_matrices.size();
        if (affine_matrices.size()!=0){
            for (int i=0; i<affine_matrices.size(); i++){
                // yInfo()<<"outside"<<i; 
                if (i%2==0){
                    cv::Mat mat_left = affine_matrices[i];
                    cv::Mat mat_right = affine_matrices[i+1];
                    cv::Mat current_matrix; 
                    cv::hconcat(mat_left, mat_right, current_matrix);
                    couple_matrix.push_back(current_matrix);
                }

            }
            // yInfo()<<"--------------------------------"<<couple_matrix.size();

            if (couple_matrix.size() != 0)
                cv::vconcat(couple_matrix, concat_affines); 
        }
        
        cv::imshow("affine remap", concat_affines);
        affine_matrices.clear(); 
        couple_matrix.clear(); 

    }

    void setEROS(const cv::Mat &eros)
    {
        eros_resized = 0; 
        // cv::GaussianBlur(eros, eros_filtered, cv::Size(5, 5), 0);
        if (median_blur_active){
            cv::Mat eros_blurred1; 
            cv::medianBlur(eros, eros_blurred1, median_blur_eros);
            cv::GaussianBlur(eros_blurred1, eros_filtered, cv::Size(gaussian_blur_eros, gaussian_blur_eros), 0);
        }
        else{
            cv::GaussianBlur(eros, eros_filtered, cv::Size(gaussian_blur_eros, gaussian_blur_eros), 0);
        }

        eros_filtered(roi_around_shape).copyTo(eros_tracked);
        eros_tracked.convertTo(eros_tracked_64f, CV_64F, 0.003921569); 

        // yInfo()<<"eros sizes="<<eros_tracked.cols<<eros_tracked.rows; 

        cv::resize(eros_tracked_64f, eros_resized(o_proc_roi), o_proc_roi.size(), 0, 0, cv::INTER_LINEAR);
    }

    double similarity_score(const cv::Mat &observation, const cv::Mat &expectation) {
        static cv::Mat muld;
        muld = expectation.mul(observation);
        return cv::sum(cv::sum(muld))[0];
    }

    void performComparisons(){

        for (int affine = 0; affine < affine_info.size(); affine++) {
            // affine_info[affine].score = similarity_score(eros_tracked_64f, affine_info[affine].warped_img);
            affine_info[affine].score = similarity_score(eros_resized, affine_info[affine].warped_img);
            scores_vector.push_back(affine_info[affine].score);
            // cv::imshow("affine"+std::to_string(affine), affine_info[affine].warped_img);
        }
    }

    void updateState(){

        int best_score_index = max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        double best_score = *max_element(scores_vector.begin(), scores_vector.end());
        // yInfo() << scores_vector;
        // yInfo() << "highest score =" << best_score_index << best_score;
        scores_vector.clear();

        if (best_score_index == 0)
            state[0] += translation;
        else if (best_score_index == 1)
            state[0] -= translation;
        else if (best_score_index == 2)
            state[1] += translation;
        else if (best_score_index == 3)
            state[1] -= translation;
        else if (best_score_index == 4)
            state[2] += angle;
        else if (best_score_index == 5)
            state[2] -= angle;
        else if (best_score_index == 6)
            state[3] *= nscale;
        else if (best_score_index == 7)
            state[3] *= pscale;
    }

    void updateStateAll(){

        // int best_score_index = max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        // double best_score = *max_element(scores_vector.begin(), scores_vector.end());
        // yInfo() << scores_vector;
        // yInfo() << "highest score =" << best_score_index << best_score;
        double no_motion = scores_vector[8];
        if(scores_vector[0] > no_motion) state[0] += translation;
        if(scores_vector[1] > no_motion) state[0] -= translation;
        if(scores_vector[2] > no_motion) state[1] += translation;
        if(scores_vector[3] > no_motion) state[1] -= translation;
        if(scores_vector[4] > no_motion) state[2] += angle;
        if(scores_vector[5] > no_motion) state[2] -= angle;
        if(scores_vector[6] > no_motion) state[3] *= nscale;
        if(scores_vector[7] > no_motion) state[3] *= pscale;

        scores_vector.clear();
    }

    void updateStateGT(std::vector<std::vector<double>> gt_values, int counter){

        yInfo()<<gt_values[counter][0]<<" "<<gt_values[counter][1]<<" "<<gt_values[counter][2]<<" "<<gt_values[counter][3]; 
        state[0] += gt_values[counter][0]; 
        state[1] += gt_values[counter][1];
        state[2] += gt_values[counter][2];
        state[3] *= gt_values[counter][3];

        yInfo()<<"updated state"; 

    }

    void create_map_x(double dp, int affine_n){
        //x shift by dp
        for(int x = 0; x < proc_size.width; x++) {
            for(int y = 0; y < proc_size.height; y++) {
                //positive
                prmx.at<float>(y, x) = x-dp;
                prmy.at<float>(y, x) = y;
                //negative
                nrmx.at<float>(y, x) = x+dp;
                nrmy.at<float>(y, x) = y;
            }
        }
        if (affine_n == 0){
            cv::convertMaps(prmx, prmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = dp; 
        }
        else{
            cv::convertMaps(nrmx, nrmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = -dp;
        }
        affine_info[affine_n].axis = affine_n;
    }

    void create_map_y(double dp, int affine_n){
        for(int x = 0; x < proc_size.width; x++) {
            for(int y = 0; y < proc_size.height; y++) {
                //positive
                prmx.at<float>(y, x) = x;
                prmy.at<float>(y, x) = y-dp;
                //negative
                nrmx.at<float>(y, x) = x;
                nrmy.at<float>(y, x) = y+dp;
            }
        }

        if (affine_n == 2){
            cv::convertMaps(prmx, prmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = dp; 
        }
        else{
            cv::convertMaps(nrmx, nrmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = -dp;
        }
        affine_info[affine_n].axis = affine_n; 
    }

    void create_map_scale(double sc, int affine_n){
        double cy = proc_size.height * 0.5;
        double cx = proc_size.width  * 0.5;
        double palpha = sc;
        double nalpha = sc;
        for(int x = 0; x < proc_size.width; x++) {
            for(int y = 0; y < proc_size.height; y++) {
                if (affine_n == 6){
                    double pdx = (1-palpha)*cx;
                    double pdy = (1-palpha)*cy; 
                    prmx.at<float>(y, x) = palpha*x + pdx;
                    prmy.at<float>(y, x) = palpha*y + pdy;
                }
                if (affine_n == 7){
                    double ndx = (1-nalpha)*cx;
                    double ndy = (1-nalpha)*cy; 
                    nrmx.at<float>(y, x) = nalpha*x + ndx;
                    nrmy.at<float>(y, x) = nalpha*y + ndy; 
                }
            }
        }
        if (affine_n == 6){
            cv::convertMaps(prmx, prmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = sc; 
        }
        else{
            cv::convertMaps(nrmx, nrmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = sc;
        }
        affine_info[affine_n].axis = affine_n;
    }

    void create_map_z(double sc, int affine_n)
    {
        double scaling_fac; 
        if(affine_n == 6){
            scaling_fac = sc - 1;
        }
        else{
            scaling_fac = 1 - sc;
        }
        double cy = proc_size.height * 0.5;
        double cx = proc_size.width  * 0.5;
        for(int x = 0; x < proc_size.width; x++) {
            for(int y = 0; y < proc_size.height; y++) {
                double dx = -(x-cx) * scaling_fac;
                double dy = -(y-cy) * scaling_fac;
                //positive
                prmx.at<float>(y, x) = x - dx;
                prmy.at<float>(y, x) = y - dy;
                //negative
                nrmx.at<float>(y, x) = x + dx;
                nrmy.at<float>(y, x) = y + dy;
            }
        }
        if (affine_n == 6){
            cv::convertMaps(prmx, prmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = scaling_fac; 
        }
        else{
            cv::convertMaps(nrmx, nrmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = scaling_fac;
        }
        affine_info[affine_n].axis = affine_n;
    }


    void create_map_rot(double angle, int affine_n){
        double cy = proc_size.height * 0.5;
        double cx = proc_size.width  * 0.5;
        double angle_rad = angle*M_PI/180;
        double palpha = cos(angle_rad);
        double pbeta = sin(angle_rad);
        double nalpha = cos(angle_rad);
        double nbeta = -sin(angle_rad);
        // double theta = atan2(angle_rad, std::max(proc_size.width, proc_size.height)*0.5);
        // yInfo()<<angle<<angle_rad; 
        for(int x = 0; x < proc_size.width; x++) {
            for(int y = 0; y < proc_size.height; y++) {
                double pdx = (1-palpha)*cx - pbeta*cy;
                double pdy = pbeta*cx + (1-palpha)*cy; 
                double ndx = (1-nalpha)*cx - nbeta*cy;
                double ndy = nbeta*cx + (1-nalpha)*cy; 
                //positive
                nrmx.at<float>(y, x) = palpha*x + pbeta*y + pdx;
                nrmy.at<float>(y, x) = -pbeta*x + palpha*y + pdy;
                //negative
                prmx.at<float>(y, x) = nalpha*x + nbeta*y + ndx;
                prmy.at<float>(y, x) = -nbeta*x + nalpha*y + ndy; 
            }
        }
        if (affine_n == 4){
            cv::convertMaps(prmx, prmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            // affine_info[affine_n].delta = theta;
        }
        else{
            cv::convertMaps(nrmx, nrmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            // affine_info[affine_n].delta = -theta;
        }
        
        affine_info[affine_n].axis = affine_n;
    }

    void create_maps(){
        create_map_x(translation, 0); 
        create_map_x(translation, 1);
        create_map_y(translation, 2);
        create_map_y(translation, 3);
        create_map_rot(angle, 4);
        create_map_rot(angle, 5);
        create_map_scale(pscale, 6);
        create_map_scale(nscale, 7);
    }


};