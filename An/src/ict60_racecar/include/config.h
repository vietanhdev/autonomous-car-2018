#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include "yaml-cpp/yaml.h"


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


};

#endif