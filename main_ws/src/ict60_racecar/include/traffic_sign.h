#ifndef TRAFFIC_SIGN_H
#define TRAFFIC_SIGN_H
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>

// struct TrafficSignStructure {
//         int id;
//         cv::Rect rect;
//         TrafficSignStructure(int id, cv::Rect rect) : id(id), rect(rect)
//         {
//         }
// };

// typedef struct TrafficSignStructure TrafficSign; 

class TrafficSign {
    public:
        int id;
        cv::Rect rect;
        std::chrono::time_point<std::chrono::high_resolution_clock> observe_time;

        TrafficSign(int id, cv::Rect rect) {
            this->id = id;
            this->rect = rect;
            this->observe_time = std::chrono::system_clock::now();
        }
};

class TrafficSignDetector {

    public:

    cv::Ptr<cv::ml::SVM> svm;

    cv::HOGDescriptor hog;
    std::string color_file;
    std::string svm_file;

    typedef struct ColorRangeStruct {
        cv::Scalar begin;
        cv::Scalar end;
        ColorRangeStruct(double h1, double s1, double v1,
        double h2, double s2, double v2) : begin(cv::Scalar(h1, s1, v1)), end(cv::Scalar(h2, s2, v2)) {}
    } ColorRange;

    std::vector<ColorRange> color_ranges;

    TrafficSignDetector();

    /*!
    * \brief Enlarge an ROI rectangle by a specific amount if possible 
    * \param frm The image the ROI will be set on
    * \param boundingBox The current boundingBox
    * \param padding The amount of padding around the boundingbox
    * \return The enlarged ROI as far as possible
    */
    cv::Rect enlargeROI(cv::Mat frm, cv::Rect boundingBox, int padding);

    void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);

    void getSVMParams(cv::ml::SVM *svm);

    std::vector<cv::Rect> detect(const cv::Mat & input);

    // Receive list of detected objects and classify them
    // Return: sign type id. Return -1 if the object is not a traffic sign
    // hogDetectorsPath: path to HOG detectors
    void classify(cv::Mat & img,
                std::vector<cv::Rect>& boundaries,
                cv::Ptr<cv::ml::SVM> & svm,
                std::vector<TrafficSign> & classification_results
        );
    
    void recognize(cv::Mat & input, std::vector<TrafficSign> & classification_results);

    void readColorFile();

};



#endif