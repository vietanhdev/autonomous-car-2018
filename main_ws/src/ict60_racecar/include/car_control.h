#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include "timer.h"

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>

#include "config.h"
#include "road.h"
#include "lane_detector.h"
#include "obstacle_detector.h"
#include "traffic_sign.h"

// #include "PID.h"



class CarControl 
{
public:

    bool debug_flag = false;

    float MAX_SPEED = 60;
    float MAX_ANGLE = 60;

    CarControl();
    ~CarControl();
    void driverCar(float speed_data, float angle_data);
    void driverCar(Road & road, const std::vector<TrafficSign> & traffic_signs);

private:

    Config config;


    int obstacle_avoid_coeff = 0;
    Timer::time_point_t obstacle_avoiding_time_point;


    int last_sign_id = -1;
    Timer::time_point_t last_sign_time_point;
    bool prepare_to_turn = false;
    bool is_turning = false;
    int success_turning_times = 0;
    int turning_coeff = 0;

    Timer::time_point_t turning_time_point;

    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    // Kalman filter
    cv::KalmanFilter KF;
    cv::Mat state; /* (phi, delta_phi) */
    cv::Mat process_noise;
    cv::Mat measurement;


    // TODO: complete this function
    double kalmanFilterAngle(double angle) {

        return angle;

    }


    float delta_to_angle_coeff = -0.5;

    void readConfig();

};

#endif