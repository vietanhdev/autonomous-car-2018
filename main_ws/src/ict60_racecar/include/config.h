#ifndef CONFIG_H
#define CONFIG_H

#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <ros/console.h>


class Config {

    public:

    YAML::Node config;

    Config() {

        std::string path = ros::package::getPath(getROSPackage()) + std::string("/data/config.yaml");

        // Read config file
        config = YAML::LoadFile(path);

    }


    Config(std::string config_file_name) {

        std::string path = ros::package::getPath(getROSPackage()) + std::string("/data/" + config_file_name);

        // Read config file
        config = YAML::LoadFile(path);

    }

    // Copy constructor 
    Config(const Config &c) {
        config = YAML::Clone(config);
    } 

    static const std::string& getROSPackage() {
        static std::string ROS_PACKAGE("ict60_racecar");
        return ROS_PACKAGE;
    }

    std::string getTeamName() {
        return get<std::string>("team_name");
    }

    template <class T>
    T get(std::string key) {
        return config[key].as<T>();
    }

    cv::Point getPoint(std::string key) {
        std::string point_str = config[key].as<std::string>();
        std::vector<int> nums =  extractIntegers(point_str);

        ROS_ASSERT_MSG(nums.size() == 2, "Error  on reading %s", key.c_str());

        return cv::Point(nums[0], nums[1]);
    }

    cv::Size getSize(std::string key) {
        std::string size_str = config[key].as<std::string>();
        std::vector<int> nums =  extractIntegers(size_str);

        ROS_ASSERT_MSG(nums.size() == 2, "Error  on reading %s", key.c_str());

        return cv::Size(nums[0], nums[1]);
    }

    cv::Scalar getScalar3(std::string key) {
        std::string size_str = config[key].as<std::string>();
        std::vector<int> nums =  extractIntegers(size_str);

        ROS_ASSERT_MSG(nums.size() == 3, "Error  on reading %s", key.c_str());

        return cv::Scalar(nums[0], nums[1], nums[2]);
    }

    static std::vector<int> extractIntegers(std::string str)  { 
        std::stringstream ss;  
        std::vector<int> results;
    
        /* Storing the whole string into string stream */
        ss << str; 
    
        /* Running loop till the end of the stream */
        std::string temp; 
        int found; 
        while (!ss.eof()) { 
    
            /* extracting word by word from stream */
            ss >> temp; 
    
            /* Checking the given word is integer or not */
            if (std::stringstream(temp) >> found) 
                results.push_back(found); 
    
            /* To save from space at the end of string */
            temp = ""; 
        }

        return results;
    } 


};

#endif