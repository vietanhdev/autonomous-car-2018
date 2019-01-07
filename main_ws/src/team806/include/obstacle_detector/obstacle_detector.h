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

using namespace std;
using namespace cv;

class DetectedObject {
   public:
    int label;
    int seen_times;
    int disappeared_times;
    cv::Rect last_position;

    DetectedObject(int label, cv::Rect position) {
        this->label = label;
        this->last_position = position;
        seen_times = 1;
        disappeared_times = 0;
    }
};

class ObstacleDetector {
   public:
    bool debug_flag = false;
    std::shared_ptr<Config> config;

    std::vector<cv::Mat> obstacles;
    std::vector<DetectedObject> detected_objects;

    ObstacleDetector() {
        // Load the config
        config = Config::getDefaultConfigInstance();

        // Load all obstacle models
        std::string obstacles_folder_path =
            ros::package::getPath(config->getROSPackage()) +
            config->get<std::string>("obstacles_folder");

        // List all image files in obstacles folder => read images into obstacle
        // database Currently only support jpg, png and bmp
        std::vector<std::string> obstacle_files;
        DIR *dir;
        char buffer[PATH_MAX + 1];
        struct dirent *ent;
        if ((dir = opendir(obstacles_folder_path.c_str())) != NULL) {
            /* print all the files and directories within directory */
            while ((ent = readdir(dir)) != NULL) {
                if (strcmp(ent->d_name, ".") != 0 &&
                    strcmp(ent->d_name, "..") != 0) {
                    
                    std::string file_name(ent->d_name);
                    std::string file_path = obstacles_folder_path + "/" + file_name;

                    // Check file extension
                    std::string file_ext =
                        ObstacleDetector::getFileExt(file_path);
                    std::transform(file_ext.begin(), file_ext.end(),
                                   file_ext.begin(), ::tolower);
                    if (file_ext == "jpg" || file_ext == "png" ||
                        file_ext == "bmp") {
                        std::cout << "Loading Obstacle: " << file_path
                                  << std::endl;

                        cv::Mat templ = cv::imread(file_path, 1);
                        if (templ.empty()) {
                            std::cerr << "Could not read template file: " << file_path << std::endl;
                        } else {
                            this->obstacles.push_back(templ);
                        }

                    }
                }
            }
            closedir(dir);
        } else {
            /* could not open directory */
            perror("");
            return;
        }
    }

    void updateDetectedList(std::vector<DetectedObject> &objects,
                            std::vector<int> &new_object_labels,
                            const std::vector<cv::Rect> &new_object_pos) {
        // We use this flag to determine if the new object is processed (merge
        // with old detected one) or not
        std::vector<bool> processed(new_object_labels.size());
        std::fill(processed.begin(), processed.end(), false);

        // Loop through detected objects
        for (int i = 0; i < objects.size(); ++i) {
            // Old object is updated or not
            bool updated = false;

            for (int j = 0; j < new_object_labels.size(); ++j) {
                if (!processed[j]) {
                    // New position matches old object
                    if (objects[i].label == new_object_labels[j] &&
                        (objects[i].last_position & new_object_pos[j]).area() >
                            0) {
                        objects[i].last_position = new_object_pos[j];
                        ++objects[i].seen_times;
                        objects[i].disappeared_times = 0;
                        processed[j] = true;
                        break;
                    }
                }
            }

            if (!updated) {
                ++objects[i].disappeared_times;
            }
        }

        // Add not-processed new object as new element of object array
        for (int i = 0; i < new_object_labels.size(); ++i) {
            if (!processed[i]) {
                objects.push_back(
                    DetectedObject(new_object_labels[i], new_object_pos[i]));
            }
        }
    }

    // Detect the obstacles
    // Return the number of obstacles in the input image
    int detect(const cv::Mat &img,
               std::vector<cv::Rect> &detected_obstacle_bounds) {
        std::vector<int> new_object_labels;
        std::vector<cv::Rect> new_objects;

        // Detect each object in the image
        for (int i = 0; i < obstacles.size(); ++i) {
            cv::Mat obstacle_templ = obstacles[i];
            std::vector<cv::Rect> found_obstacles;

            // Do template matching for object
            if (matching(img, obstacle_templ, found_obstacles)) {
                new_objects.insert(
                    new_objects.end(),
                    std::make_move_iterator(found_obstacles.begin()),
                    std::make_move_iterator(found_obstacles.end()));
                for (int k = 0; k < found_obstacles.size(); ++k)
                    new_object_labels.push_back(i);
            }
        }

        updateDetectedList(detected_objects, new_object_labels, new_objects);

        // Return the result
        detected_obstacle_bounds.clear();
        for (int i = 0; i < detected_objects.size(); ++i) {
            if (detected_objects[i].disappeared_times < 2 &&
                detected_objects[i].seen_times > 1) {
                detected_obstacle_bounds.push_back(
                    detected_objects[i].last_position);
            }
        }

        // Draw debug image
        if (debug_flag) {
            cv::Mat draw = img.clone();

            for (int i = 0; i < detected_objects.size(); ++i) {
                if (detected_objects[i].disappeared_times < 2 &&
                    detected_objects[i].seen_times > 1) {
                    cv::rectangle(draw, detected_objects[i].last_position,
                                  cv::Scalar(0, 255, 0), 2, 8, 0);
                    cv::putText(draw, std::to_string(detected_objects[i].label),
                                detected_objects[i].last_position.tl(),
                                FONT_HERSHEY_COMPLEX, 2, Scalar(255, 255, 255),
                                1);
                }
            }

            cv::imshow("Object Detector Debug", draw);
            cv::waitKey(1);
        }

        // cout << "DETECTED OBSTACLES: " << detected_obstacle_bounds.size() << endl;

        return detected_obstacle_bounds.size();
    }

    // Matching object using template matching
    bool matching(const Mat &img, const Mat &templ,
                  std::vector<cv::Rect> &rects) {
        // Clear the result
        rects.clear();

        // Matching object using template matching
        int match_method = TM_CCOEFF_NORMED;

        /// Create the result matrix
        int result_cols = img.cols - templ.cols + 1;
        int result_rows = img.rows - templ.rows + 1;

        cv::Mat result;
        result.create(result_rows, result_cols, CV_32FC1);

        matchTemplate(img, templ, result, match_method);

        /// Localizing the best match with minMaxLoc
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Point matchLoc;

        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

        /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For
        /// all the other methods, the higher the better
        if (match_method == CV_TM_SQDIFF ||
            match_method == CV_TM_SQDIFF_NORMED) {
            matchLoc = minLoc;
        } else {
            matchLoc = maxLoc;
        }

        /// Show me what you got
        if (maxVal > 0.8) {
            cout << "MATCHED" << endl;
            rects.push_back(cv::Rect(matchLoc, Point(matchLoc.x + templ.cols,
                                                     matchLoc.y + templ.rows)));
            return true;
        }

        return false;
    }

    static std::string getFileExt(const std::string &s) {
        size_t i = s.rfind('.', s.length());
        if (i != std::string::npos) {
            return (s.substr(i + 1, s.length() - i));
        }

        return ("");
    }

    static void printLineDiff(const std::vector<cv::Point> &line) {
        for (int i = line.size() - 2; i >= 0; --i) {
            std::cout << (int)abs(line[i].x - line[i + 1].x) << ",";
        }
        std::cout << std::endl;
    }
};

#endif