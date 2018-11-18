#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "timer.h"
#include "road.h"
#include "lane_detector.h"
#include "obstacle_detector.h"
#include "traffic_sign.h"
#include "traffic_sign_detector.h"
#include "traffic_sign_detector_2.h"
#include "car_control.h"

#define ROS_PACKAGE "ict60_racecar"
#define TEAM "Team1"
#define SHOW_ORIGIN_IMAGE false
#define DEBUG_FIND_LANES false
#define DEBUG_CAR_CONTROL false

using namespace std;
using namespace cv;

std::shared_ptr<CarControl> car;
std::shared_ptr<LaneDetector> lane_detector;
std::shared_ptr<TrafficSignDetector> sign_detector;
Road road;

// To calculate the speed of image transporting
// ==> The speed of the hold system => Adjust the car controlling params
long long int num_of_frames = 0;
Timer::time_point_t start_time_point;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        if (SHOW_ORIGIN_IMAGE) {
            cv::imshow("View", cv_ptr->image);
            cv::waitKey(1);
        }
        

        std::vector<TrafficSign> traffic_signs;
        sign_detector->recognize(cv_ptr->image, traffic_signs);
        lane_detector->findLanes(cv_ptr->image, road);
        car->driverCar(road, traffic_signs);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    ++num_of_frames;

    double sim_speed = static_cast<double>(num_of_frames) / Timer::calcTimePassed(start_time_point) * 1000;
    cout << "Simulation speed: " << sim_speed << endl;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    lane_detector = std::shared_ptr<LaneDetector>(new LaneDetector());
    sign_detector = std::shared_ptr<TrafficSignDetector>(new TrafficSignDetector());
    car = std::shared_ptr<CarControl>(new CarControl(TEAM));


    // SET DEBUG OPTIONS
    lane_detector->debug_flag = DEBUG_FIND_LANES;
    car->debug_flag = DEBUG_CAR_CONTROL;

    cv::startWindowThread();

    start_time_point = Timer::getCurrentTime();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(std::string(TEAM)+"_image", 1, imageCallback);

    ros::spin();

    cv::destroyAllWindows();
}