#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <iostream> 
#include <sstream> 
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include "config.h"
#include "road.h"



class LaneDetector {

    public:

    bool debug_flag = false;

    Config config;

    // ==== CONFIGURATION ====

    cv::Size img_size;

    // ** Floodfill
    cv::Scalar floodfill_lo;
    cv::Scalar floodfill_hi;
    std::vector<cv::Point> floodfill_points;

    int lane_area;

    // ** Perspective transform
    cv::Mat perspective_matrix_;
	cv::Mat inverse_perspective_matrix_;
    cv::Size perspective_img_size;
    cv::Mat interested_area; // Area of the transformed image appearing in the original image


    void initConfig();

    // ** Initialize perspective transform matrices
    void initPerspectiveTransform();

    // ** Constructor
    LaneDetector();

    static cv::Point getNullPoint();

    /*--- Floodfill ---*/
    void laneFloodFill(const cv::Mat & img, cv::Mat & dst, cv::Point start_point);

    void laneFloodFillPoints(const cv::Mat & img, cv::Mat & mask);

    void doCannyEdges(const cv::Mat & img, cv::Mat & mask);

    int getPerspectiveMatrix(const std::vector<cv::Point2f> corners_source, const std::vector<cv::Point2f> corners_trans);

    bool findLaneMask(const cv::Mat & img, cv::Mat & mask);

    void perspectiveTransform(const cv::Mat & src, cv::Mat & dst);

    void removeCenterLaneLine(const cv::Mat & mask, cv::Mat output_mask);

    void findEdgePoints(const cv::Mat & mask, size_t row, cv::Point & left_point, cv::Point & right_point);

    void findLaneEdges(const cv::Mat & img, Road & road);

    void findLanes(const cv::Mat & input, Road & road);

};

#endif