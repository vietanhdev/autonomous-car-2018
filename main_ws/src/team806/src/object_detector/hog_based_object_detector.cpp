#include "hog_based_object_detector.h"

using namespace std;
using namespace cv;

HogBasedObjectDetector::HogBasedObjectDetector(
    DetectedObject::ObjectLabel label, cv::HOGDescriptor hog, cv::Size winstride) {
    this->label = label;
    this->hog = hog;
    this->winstride = winstride;
}

HogBasedObjectDetector::HogBasedObjectDetector(
    DetectedObject::ObjectLabel label, const std::string &hog_file, cv::Size winstride) {
    hog.load(hog_file);
    this->winstride = winstride;
}

HogBasedObjectDetector::HogBasedObjectDetector(
    DetectedObject::ObjectLabel label, const std::string &hog_file,
    double threshold, cv::Size winstride) {

    hog.load(hog_file);
    this->threshold = threshold;
    this->winstride = winstride;

}

// Detect the objects
// Return the number of objects in the input image
int HogBasedObjectDetector::detect(
    const cv::Mat &img, std::vector<DetectedObject> &detected_objects) {

    // Clear result vector
    detected_objects.clear();
    if (img.empty()) return 0;

    std::vector<cv::Point> foundPoints;
    std::vector<cv::Rect> foundLocations;
    std::vector<double> weights;


    hog.detect(img, foundPoints, weights, this->threshold, this->winstride);

    for (int i = 0; i < foundPoints.size(); ++i) {
        foundLocations.push_back(
        cv::Rect(foundPoints[i],
            cv::Point(foundPoints[i].x + 32, foundPoints[i].y + 32))
        );
    }

    if (!foundLocations.empty()) {
        for (int i = 0; i < foundLocations.size(); ++i) {
            // TODO: Add debug
            // rectangle(frame, foundLocations[i], Scalar(0,255,0), 2);
            // putText(frame, std::to_string(weights[i]),
            // foundLocations[i].tl(), FONT_HERSHEY_COMPLEX, 0.5,
            // Scalar(0,255,0), 1);

            detected_objects.push_back(
                DetectedObject(this->label, foundLocations[i], weights[i]));
        }
    }

    return detected_objects.size();
}