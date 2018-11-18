#include "timer.h"
#include "traffic_sign.h"
#include "traffic_sign_detector.h"

std::string video_path = "../../../../videos/outcpp.avi";

int main(int argc, char** argv){

    cv::VideoCapture cap(video_path);
    cv::Mat frame;

    TrafficSignDetector* traffic_sign_detector = new TrafficSignDetector();

    while (true) {
        cap >> frame;
        if(frame.empty()) break;

        std::vector<TrafficSign> traffic_signs;
        traffic_sign_detector->trafficSignDetect(frame, traffic_signs);

        imshow("video", frame);
        char key = (char) cv::waitKey(20);
        if (key == 'q' || key == 27) break;  
    }

    frame.release();
    
    cv::destroyAllWindows();

	return 0;
}