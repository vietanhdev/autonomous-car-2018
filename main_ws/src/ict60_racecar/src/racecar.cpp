#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


#include "config.h"
#include "timer.h"
#include "road.h"
#include "lane_detector.h"
#include "obstacle_detector.h"
#include "traffic_sign.h"
#include "traffic_sign_detector.h"
#include "traffic_sign_detector_2.h"
#include "car_control.h"

using namespace std;
using namespace cv;

bool show_origin_image;
bool use_traffic_sign_detector_2 = false;

std::shared_ptr<CarControl> car;
std::shared_ptr<LaneDetector> lane_detector;
std::shared_ptr<TrafficSignDetector> sign_detector;
std::shared_ptr<TrafficSignDetector2> sign_detector_2;
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
        
        if (show_origin_image) {
            cv::imshow("View", cv_ptr->image);
            cv::waitKey(1);
        }
        

        std::vector<TrafficSign> traffic_signs;
        lane_detector->findLanes(cv_ptr->image, road);

        if (!use_traffic_sign_detector_2) {
            sign_detector->recognize(cv_ptr->image, traffic_signs);
        } else {
            sign_detector_2->recognize(cv_ptr->image, traffic_signs);
        }

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
    Config config;
    ros::init(argc, argv, "image_listener");


    lane_detector = std::shared_ptr<LaneDetector>(new LaneDetector());

    use_traffic_sign_detector_2 = config.get<bool>("use_traffic_sign_detector_2");
    if (!use_traffic_sign_detector_2) {
        sign_detector = std::shared_ptr<TrafficSignDetector>(new TrafficSignDetector());
        sign_detector->debug_flag = config.get<bool>("debug_sign_detector");
    } else {
        sign_detector_2 = std::shared_ptr<TrafficSignDetector2>(new TrafficSignDetector2());
        sign_detector_2->debug_flag = config.get<bool>("debug_sign_detector");
    }

    car = std::shared_ptr<CarControl>(new CarControl());

    // SET DEBUG OPTIONS
    show_origin_image = config.get<bool>("debug_show_origin_image");
    lane_detector->debug_flag = config.get<bool>("debug_lane_detector");
    car->debug_flag = config.get<bool>("debug_car_control");
    


    cv::startWindowThread();

    start_time_point = Timer::getCurrentTime();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(config.getTeamName()+"_image", 1, imageCallback);

    ros::spin();

    cv::destroyAllWindows();
}