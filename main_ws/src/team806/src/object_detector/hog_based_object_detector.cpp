#include "hog_based_object_detector.h"

using namespace std;
using namespace cv;

HogBasedObjectDetector::HogBasedObjectDetector(
    DetectedObject::ObjectLabel label, cv::HOGDescriptor hog) {
    this->label = label;
    this->hog = hog;
    winstride = cv::Size(4, 4);
}

HogBasedObjectDetector::HogBasedObjectDetector(
    DetectedObject::ObjectLabel label, const std::string &hog_file) {
    hog.load(hog_file);
    winstride = cv::Size(4, 4);
}

HogBasedObjectDetector::HogBasedObjectDetector(
    DetectedObject::ObjectLabel label, const std::string &hog_file,
    double threshold) {
    std::cout << hog_file << std::endl;

    // hog.load(hog_file);
    hog = cv::HOGDescriptor(cv::Size(32, 32),  // winSize
                            cv::Size(8, 8),    // blocksize
                            cv::Size(4, 4),    // blockStride,
                            cv::Size(2, 2),    // cellSize,
                            9,                 // nbins,
                            1,                 // derivAper,
                            -1,                // winSigma,
                            0,                 // histogramNormType,
                            0.2,               // L2HysThresh,
                            1,                 // gamma correction,
                            64,                // nlevels=64
                            1                  //_signedGradient = true
    );
    std::cout << hog.load(
                     "/mnt/DATA/Works/CuocDuaSo/main_ws/src/team806/data/"
                     "object_hog_files/object1.yml")
              << std::endl;
    std::cout << hog.winSize;
    this->threshold = threshold;
    winstride = cv::Size(4, 4);
}

// Detect the objects
// Return the number of objects in the input image
int HogBasedObjectDetector::detect(
    const cv::Mat &img, std::vector<DetectedObject> &detected_objects) {
    if (img.empty()) return 0;

    // Clear result vector
    detected_objects.clear();

    std::vector<cv::Point> foundPoints;
    std::vector<cv::Rect> foundLocations;
    std::vector<double> weights;

    // cv::imwrite("debug.png", img);
    cv::Mat gray;
    cv::cvtColor(img, gray, COLOR_BGR2GRAY);
    // cv::imwrite("gray.png", gray);
    cv::resize(gray, gray, cv::Size(320, 240));
    // std::cout << " winSize: " << hog.winSize;

    hog.detectMultiScale(gray, foundLocations, weights, 2.3, winstride);

    // for (int i = 0; i < foundPoints.size(); ++i) {
    //     foundLocations.push_back(
    //     cv::Rect(foundPoints[i],
    //         cv::Point(foundPoints[i].x + 32, foundPoints[i].y + 32))
    //     );
    // }

    if (!foundLocations.empty()) {
        for (int i = 0; i < foundLocations.size(); ++i) {
            // TODO: Add debug
            // rectangle(frame, foundLocations[i], Scalar(0,255,0), 2);
            // putText(frame, std::to_string(weights[i]),
            // foundLocations[i].tl(), FONT_HERSHEY_COMPLEX, 0.5,
            // Scalar(0,255,0), 1);

            detected_objects.push_back(
                DetectedObject(this->label, foundLocations[i]));
        }
    }

    return detected_objects.size();
}