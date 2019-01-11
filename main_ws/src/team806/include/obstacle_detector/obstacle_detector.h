#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <dirent.h>
#include <limits.h>
#include <ros/package.h>
#include <algorithm>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>
#include "config.h"
#include "image_publisher.h"

class DetectedObject {
   public:
    int label;
    int seen_times;
    int disappeared_times;
    cv::Rect last_position;

    DetectedObject(int label, cv::Rect position);
};

class ObstacleDetector : ImagePublisher {
   public:
    bool debug_flag = false;
    image_transport::Publisher debug_img_publisher;

    std::shared_ptr<Config> config;

    std::vector<cv::Mat> obstacles;
    std::vector<DetectedObject> detected_objects;

    ObstacleDetector();

    // Merge new detected object into object list (std::vector<DetectedObject> detected_objects)
    void updateDetectedList(std::vector<DetectedObject> &objects,
                        std::vector<int> &new_object_labels,
                        const std::vector<cv::Rect> &new_object_pos);

    // Detect the obstacles
    // Return the number of obstacles in the input image
    int detect(const cv::Mat &img,
           std::vector<cv::Rect> &detected_obstacle_bounds);

    // Matching object using template matching
    bool matching(const cv::Mat &img, const cv::Mat &templ,
                  std::vector<cv::Rect> &rects);

    // Get file extension from filepath/filename
    static std::string getFileExt(const std::string &s);


    // TODO: LEGACY way to detect obstacle. Remove in the future
    static void printLineDiff(const std::vector<cv::Point> &line) {
        for (int i = line.size() - 2; i >= 0; --i) {
            std::cout << (int)abs(line[i].x - line[i + 1].x) << ",";
        }
        std::cout << std::endl;
    }
};

#endif