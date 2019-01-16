#ifndef DETECTED_OBJECT_H
#define DETECTED_OBJECT_H

#include <ros/package.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class DetectedObject {

   public:
    enum ObjectLabel { TURN_LEFT_SIGN, TURN_RIGHT_SIGN, OBJECT_1, OBJECT_2 };

    ObjectLabel label;
    cv::Rect rect;
    double weight; 

    DetectedObject(ObjectLabel label, cv::Rect rect);
    DetectedObject(ObjectLabel label, cv::Rect rect, double weight);
};

#endif