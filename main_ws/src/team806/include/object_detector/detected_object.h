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
    enum ObjectLabel { TURN_LEFT_SIGN = 1, TURN_RIGHT_SIGN = 2, OBJECT_1 = 3, OBJECT_2 = 4 };

    ObjectLabel label;
    cv::Rect rect;
    double weight;

    unsigned int hit_history = 1; // Hit/miss history in binary

    DetectedObject(ObjectLabel label, cv::Rect rect);
    DetectedObject(ObjectLabel label, cv::Rect rect, double weight);
};

#endif