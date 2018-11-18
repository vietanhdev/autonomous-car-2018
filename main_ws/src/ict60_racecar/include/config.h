#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include "yaml-cpp/yaml.h"

class ConfigItem {

    public:

    std::string key;
    std::string value;
    ConfigItem(std::string key, std::string value) {
        this->key = key;
        this->value = value;
    }

};

class Config {

    public:

    YAML::Node config;

    Config() {

        std::string path = ros::package::getPath(getROSPackage()) + std::string("/data/config.yaml");

        // Read config file
        config = YAML::LoadFile(path);

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