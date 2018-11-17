#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "timer.h"

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>


#include "road.h"
#include "lane_detector.h"
#include "obstacle_detector.h"
#include "traffic_sign.h"
#include "traffic_sign_detector.h"

class CarControl 
{
public:

    bool debug_flag = false;

    const int MAX_SPEED = 60;
    const int MAX_ANGLE = 60;

    CarControl(std::string team);
    ~CarControl();
    void driverCar(float speed_data, float angle_data);
    void driverCar(Road & road, const std::vector<TrafficSign> & traffic_signs);

private:

    std::string team_name;

    int obstacle_avoid_coeff = 0;
    Timer::time_point_t obstacle_avoiding_time_point;


    int last_sign_id = -1;
    bool prepare_to_turn = false;

    bool is_turning = false;
    int success_turning_times = 0;
    int turning_coeff = 0;

    Timer::time_point_t turning_time_point;

    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

};

#endif