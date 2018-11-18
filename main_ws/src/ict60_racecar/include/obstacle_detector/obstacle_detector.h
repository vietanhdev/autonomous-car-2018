#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H


#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class ObstacleDetector {

    public:

    static void printLineDiff(const std::vector<cv::Point> & line) {
        for (int i = line.size()-2; i >=  0; --i) {
            std::cout << (int)abs(line[i].x - line[i+1].x) << ",";
        }
        std::cout << std::endl;
    }

};



#endif