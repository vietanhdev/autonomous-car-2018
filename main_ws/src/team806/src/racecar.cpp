#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <thread>

#include "car_control.h"
#include "config.h"
#include "lane_detector.h"
#include "obstacle_detector.h"
#include "road.h"
#include "timer.h"
#include "traffic_sign.h"
#include "traffic_sign_detector.h"
#include "traffic_sign_detector_2.h"
#include "image_publisher.h"
#include <ros/console.h>

using namespace std;
using namespace cv;



// This flag turn to true on the first time receiving image from simulator
bool racing = false;

bool use_traffic_sign_detector_2 = false;
bool debug_show_fps = false;

std::shared_ptr<CarControl> car;
std::shared_ptr<LaneDetector> lane_detector;
std::shared_ptr<ObstacleDetector> obstacle_detector;
std::shared_ptr<TrafficSignDetector> sign_detector;
std::shared_ptr<TrafficSignDetector2> sign_detector_2;

std::shared_ptr<ImagePublisher> img_publisher;
image_transport::Publisher experiment_img_pub;
image_transport::Publisher experiment_img_pub_canny;
image_transport::Publisher experiment_img_pub_meanshift;

// ============ Current State ==================

std::mutex current_img_mutex;
cv::Mat current_img;  // current image received by car

std::mutex road_mutex;
Road road;

std::mutex traffic_signs_mutex;
std::vector<TrafficSign> traffic_signs;

std::mutex obstacles_mutex;
std::vector<cv::Rect> obstacles;

// To calculate the speed of image transporting
// ==> The speed of the hold system => Adjust the car controlling params
long long int num_of_frames = 0;
Timer::time_point_t start_time_point;

Mat markerMask;

void setLabel(cv::Mat& im, const std::string label, const cv::Point & org)
{
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::rectangle(im, org + cv::Point(0, baseline), org + cv::Point(text.width, -text.height), CV_RGB(0,0,0), CV_FILLED);
    cv::putText(im, label, org, fontface, scale, CV_RGB(0,255,0), thickness, 8);
}


void drawResultRound1() {
    while (true) {
        cv::Mat draw;

        // Copy current image to draw
        {
            std::lock_guard<std::mutex> guard(current_img_mutex);
            draw = current_img.clone();
        }

        if (draw.empty()) {
            Timer::delay(500);
            continue;
        }

        // Draw the lane center
        {
            std::lock_guard<std::mutex> guard(road_mutex);
            if (!road.middle_points.empty()) {
                circle(draw, road.middle_points[road.middle_points.size() - 1],
                       1, cv::Scalar(255, 0, 0), 3);
            }
        }

        // Draw traffic signs
        {
            std::lock_guard<std::mutex> guard(traffic_signs_mutex);
            for (int i = 0; i < traffic_signs.size(); ++i) {
                rectangle(draw, traffic_signs[i].rect, Scalar(0,0,255), 2);
                std::string text;
                if (traffic_signs[i].id == TrafficSign::SignType::TURN_LEFT) {
                    text = "turn_left";
                } else if (traffic_signs[i].id == TrafficSign::SignType::TURN_RIGHT) {
                    text = "turn_right";
                }
                setLabel(draw, text, traffic_signs[i].rect.tl());
            }
        }

        // Draw obstacles
        {
            std::lock_guard<std::mutex> guard(obstacles_mutex);
            for (int i = 0; i < obstacles.size(); ++i) {
                rectangle(draw, obstacles[i], Scalar(0,255,0), 3);
                setLabel(draw, "obstruction", obstacles[i].tl());
            }
        }

        imshow("RESULT", draw);
        waitKey(1);

        Timer::delay(100);
    }
}

Timer::time_point_t last_lane_detect_time;
void laneDetectorThread() {
    float lane_detector_fps = 40;
    float current_fps;
    Timer::time_duration_t thread_delay_time = 0;
    Timer::time_duration_t thread_delay_time_step = 4;

    while (true) {
        // Copy current image
        cv::Mat img;
        {
            std::lock_guard<std::mutex> guard(current_img_mutex);
            img = current_img.clone();
        }

        // Find lane lines
        if (!img.empty()) {
            Road new_road;
            lane_detector->findLanes(img, new_road);

            {
                std::lock_guard<std::mutex> guard(road_mutex);
                road = new_road;
            }

            Timer::delay(thread_delay_time);

            current_fps = 1000.0 / Timer::calcTimePassed(last_lane_detect_time);
            if (current_fps > lane_detector_fps) {
                thread_delay_time += thread_delay_time_step;
            } else if (current_fps < lane_detector_fps &&
                       thread_delay_time >= thread_delay_time_step) {
                thread_delay_time -= thread_delay_time_step;
            }

            // cout << "Lane Detect FPS: " << current_fps << endl;

            last_lane_detect_time = Timer::getCurrentTime();
        }
    }
}

Timer::time_point_t last_car_control_time;
void carControlThread() {
    float config_fps = 40;
    float current_fps;
    Timer::time_duration_t thread_delay_time = 0;
    Timer::time_duration_t thread_delay_time_step = 4;

    while (true) {
        {
            std::lock_guard<std::mutex> guard(road_mutex);
            std::lock_guard<std::mutex> guard2(traffic_signs_mutex);
            car->driverCar(road, traffic_signs);
        }

        Timer::delay(thread_delay_time);

        current_fps = 1000.0 / Timer::calcTimePassed(last_car_control_time);
        if (current_fps > config_fps) {
            thread_delay_time += thread_delay_time_step;
        } else if (current_fps < config_fps &&
                   thread_delay_time >= thread_delay_time_step) {
            thread_delay_time -= thread_delay_time_step;
        }

        // cout << "Car Control FPS: " << current_fps << endl;

        last_car_control_time = Timer::getCurrentTime();
    }
}

Timer::time_point_t last_trafficsign_detect_time;
void trafficSignThread() {
    float config_fps = 20;
    float current_fps;
    Timer::time_duration_t thread_delay_time = 0;
    Timer::time_duration_t thread_delay_time_step = 4;

    while (true) {

        // Copy current image
        cv::Mat img;
        {
            std::lock_guard<std::mutex> guard(current_img_mutex);
            img = current_img.clone();
        }

        // Detect traffic sign
        if (!img.empty()) {
            if (!use_traffic_sign_detector_2) {
                sign_detector->recognize(img, traffic_signs);
            } else {
                sign_detector_2->recognize(img, traffic_signs);
            }

            Timer::delay(thread_delay_time);

            current_fps =
                1000.0 / Timer::calcTimePassed(last_trafficsign_detect_time);
            if (current_fps > config_fps) {
                thread_delay_time += thread_delay_time_step;
            } else if (current_fps < config_fps &&
                       thread_delay_time >= thread_delay_time_step) {
                thread_delay_time -= thread_delay_time_step;
            }

            // cout << "Traffic sign detect FPS: " << current_fps << endl;

            last_trafficsign_detect_time = Timer::getCurrentTime();
        }

	ROS_ERROR("LOI");
    }
}


// Experiement: Meanshift

//This colors the segmentations
void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
{
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
    for( int y = 0; y < img.rows; y++ )
    {
        for( int x = 0; x < img.cols; x++ )
        {
            if( mask.at<uchar>(y+1, x+1) == 0 )
            {
                Scalar newVal( rng(256), rng(256), rng(256) );
                floodFill( img, mask, Point(x,y), newVal, 0, colorDiff, colorDiff );
            }
        }
    }
}

void meanShiftSegmentation( const cv::Mat & img, cv::Mat res )
{
    int spatialRad = 60, colorRad = 40, maxPyrLevel = 3;
    cout << "spatialRad=" << spatialRad << "; "
         << "colorRad=" << colorRad << "; "
         << "maxPyrLevel=" << maxPyrLevel << endl;
    pyrMeanShiftFiltering( img, res, spatialRad, colorRad, maxPyrLevel );
    floodFillPostprocess( res, Scalar::all(2) );
}



Timer::time_point_t last_obstacle_detect_time;
void obstacleDetectorThread() {
    float config_fps = 10;
    float current_fps;
    Timer::time_duration_t thread_delay_time = 0;
    Timer::time_duration_t thread_delay_time_step = 4;

    while (true) {
        // Copy current image
        cv::Mat img;
        {
            std::lock_guard<std::mutex> guard(current_img_mutex);
            img = current_img.clone();
        }

        // Detect obstacle
        if (!img.empty()) {
            std::vector<cv::Rect> detected_obstacles;
            obstacle_detector->detect(img, detected_obstacles);
            {
                std::lock_guard<std::mutex> guard(obstacles_mutex);
                obstacles = detected_obstacles;
            }

            Timer::delay(thread_delay_time);

            current_fps =
                1000.0 / Timer::calcTimePassed(last_obstacle_detect_time);
            if (current_fps > config_fps) {
                thread_delay_time += thread_delay_time_step;
            } else if (current_fps < config_fps &&
                       thread_delay_time >= thread_delay_time_step) {
                thread_delay_time -= thread_delay_time_step;
            }

            // cout << "Obstacle detect FPS: " << current_fps << endl;

            last_obstacle_detect_time = Timer::getCurrentTime();
        }
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try {
        if (racing == false) {
            racing = true;
            car->resetRound();
        }

        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Update current image
        {
            std::lock_guard<std::mutex> guard(current_img_mutex);
            current_img = cv_ptr->image.clone();
        }

    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                  msg->encoding.c_str());
    }

    ++num_of_frames;

    double sim_speed = static_cast<double>(num_of_frames) /
                       Timer::calcTimePassed(start_time_point) * 1000;

    if (debug_show_fps) ROS_INFO_STREAM("Simulation speed: " << sim_speed);

    
    // cout << "Simulation speed: " << sim_speed << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "team806_node");
    std::shared_ptr<Config> config = Config::getDefaultConfigInstance();

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_ERROR("Ghi Log Error");
    ROS_INFO("Ghi Log Info");

    img_publisher = std::make_shared<ImagePublisher>();
    experiment_img_pub = img_publisher->createImagePublisher("experiment_img", 1);
    experiment_img_pub_canny = img_publisher->createImagePublisher("experiment_img/canny", 1);
    experiment_img_pub_meanshift = img_publisher->createImagePublisher("experiment_img/meanshift", 1);

    debug_show_fps = config->get<bool>("debug_show_fps");

    lane_detector = std::shared_ptr<LaneDetector>(new LaneDetector());

    obstacle_detector =
        std::shared_ptr<ObstacleDetector>(new ObstacleDetector());

    use_traffic_sign_detector_2 =
        config->get<bool>("use_traffic_sign_detector_2");
    if (!use_traffic_sign_detector_2) {
        sign_detector =
            std::shared_ptr<TrafficSignDetector>(new TrafficSignDetector());
    } else {
        sign_detector_2 =
            std::shared_ptr<TrafficSignDetector2>(new TrafficSignDetector2());
    }

    car = std::shared_ptr<CarControl>(new CarControl());

    // SET DEBUG OPTIONS
    car->debug_flag = config->get<bool>("debug_car_control");

    start_time_point = Timer::getCurrentTime();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub =
        it.subscribe(config->getTeamName() + "_image", 1, imageCallback);

    std::thread round_1_result_thread(drawResultRound1);
    std::thread lane_detector_thread(laneDetectorThread);
    std::thread obstacle_detector_thread(obstacleDetectorThread);
    std::thread trafficsign_detector_thread(trafficSignThread);
    std::thread car_control_thread(carControlThread);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();


    cv::destroyAllWindows();
}
