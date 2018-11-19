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

    float MIN_SPEED = 15;
    float MAX_SPEED = 60;
    float MAX_ANGLE = 60;

    CarControl();
    ~CarControl();
    void driverCar(float speed_data, float angle_data);
    void driverCar(Road & road, const std::vector<TrafficSign> & traffic_signs);


private:

    void publishSignal(float speed_data, float angle_data);

    Config config;

    // Manage control signal publish interval
    Timer::time_point_t last_signal_publish_time_point;
    Timer::time_duration_t signal_publish_interval = 1;


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
    float middle_interested_point_pos = 0.6;
    float line_diff_to_angle_coeff = -1;

    // Minimum number of middle points found by lane detector.
    // If the number of middle points less than this value, do nothing with car controlling
    int min_num_of_middle_points = 10;

    // Minimum area of traffic sign rectangle boundary
    int min_traffic_sign_bound_area = 1000;

    // Valid duration for traffic sign recognition
    int traffic_sign_valid_duration = 3000;

    // Speed when preparing to turn (because of the apearance of a traffic sign)
    float speed_on_preparing_to_turn_trafficsign = 30;

    // Using lane area as a signal to turn
    // If the lane > this value, change the direction of the car
    int lane_area_to_turn = 18000;

    // The angle we use to change direction of the car when we meet a traffic sign
    float turning_angle_on_trafficsign = 50;

    float speed_on_turning_trafficsign = 10;

    Timer::time_duration_t turning_duration_trafficsign = 1000;



    void readConfig();

};

#endif