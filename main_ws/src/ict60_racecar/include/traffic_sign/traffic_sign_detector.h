#ifndef TRAFFIC_SIGN_DETECTOR_H
#define TRAFFIC_SIGN_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "traffic_sign.h"
#include "color_range.h"

class TrafficSignDetector {

    public:

    bool debug_flag = true;

    cv::Ptr<cv::ml::SVM> model;

    cv::HOGDescriptor hog;
    std::string color_file;
    std::string svm_file;

    std::vector<ColorRange> color_ranges;

    int size = 32;

    // use eps_diff to check similar rects of previous frame with current frame
    // (A|B).area() / [ A + B + (A&B).area()] < eps_diff
    float eps_diff = 1.5;

    std::vector<cv::Rect> prev_rects;
    std::vector<int> prev_labels;

    // hist is a queue to save size of prev_labels
    std::vector<int> hist;

    TrafficSignDetector();

    void recognize(const cv::Mat & input, std::vector<TrafficSign> & classification_results);

    // =====================================================
    // ******* HELPER ********
    // ======================================================

    void createHOG(cv::HOGDescriptor &hog, std::vector<std::vector<float>> &HOG, std::vector<cv::Mat> &cells);

    void cvtVector2Matrix(std::vector<std::vector<float>> &HOG,cv::Mat &mat);

    void readColorFile();


    // =====================================================
    // ******* SVM ********
    // ======================================================

    cv::Ptr<cv::ml::SVM> svmInit(float C, float gamma);

    void getSVMParams(cv::ml::SVM *svm);

    void svmPredict(cv::Ptr<cv::ml::SVM> svm,cv::Mat &test_response,cv::Mat &test_mat );

    // =====================================================
    // ******* DETECT ********
    // ======================================================


    void inRangeHSV(cv::Mat &img,cv::Mat &bin_img, cv::Scalar low_HSV, cv::Scalar high_HSV);

    void boundRectBinImg(cv::Mat &img, std::vector<cv::Rect> &bound_rects);

    void boundRectByColor(cv::Mat &img, std::vector<cv::Rect> &bound_rects, cv::Scalar low_HSV, cv::Scalar high_HSV);

    void mergeRects(std::vector<cv::Rect> &bound_rects);

    void extendRect(cv::Rect &rect, int extend_dist, int limit_br_x, int limit_br_y);

    bool checkSimilarityRect(cv::Rect A, cv::Rect B, float eps_diff);


    // =====================================================
    // ******* CLASSIFY ********
    // ======================================================

    void classifyRect(cv::Mat &img,
	std::vector<cv::Rect> &curr_rects, std::vector<int> &curr_labels, 
	std::vector<cv::Rect> &prev_rects, std::vector<int> &prev_labels, 
	int size, float eps_diff);

    void trafficDetect(cv::Mat &img,
	std::vector<cv::Rect> &curr_rects, std::vector<int> &curr_labels,
	std::vector<cv::Rect> &prev_rects, std::vector<int> &prev_labels,
	int size, float eps_diff,
	cv::Scalar low_HSV, cv::Scalar high_HSV);

    int classifySVM(cv::Mat &img);





};


#endif