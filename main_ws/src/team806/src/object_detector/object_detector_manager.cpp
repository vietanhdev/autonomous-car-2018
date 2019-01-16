#include "object_detector_manager.h"


ObjectDetectorManager::ObjectDetectorManager() {
    
    // Init detectors
    detectors.push_back(
        dynamic_cast<ObjectDetector*>(
            new HogBasedObjectDetector(DetectedObject::ObjectLabel::OBJECT_1, ros::package::getPath(config->getROSPackage()) + "/data/object_hog_files/object1.yml" , 2.3)
        )
    );

}


 // Detect the obstacles
// Return the number of obstacles in the input image
int ObjectDetectorManager::detect(const cv::Mat &img,
    std::vector<DetectedObject> &detected_objects) {


    // New detected objects in this global search
    std::vector<DetectedObject> new_detected_objects;

    // Search global
    if (num_of_frames_to_global_search < num_of_frames_bw_global_searchs) {
        ++num_of_frames_to_global_search;
    } else {

        // Reset the counter to a global search
        num_of_frames_to_global_search = 0;

        // Iterate between detectors and search global
        for (size_t i = 0; i < num_of_detectors_each_global_search; ++i) {
            std::vector<DetectedObject> objects;

            detectors[getDetectorIter()]->detect(img, objects);

            // Concat results into global results
            new_detected_objects.insert(
                new_detected_objects.end(),
                std::make_move_iterator(objects.begin()),
                std::make_move_iterator(objects.end())
            );

            increaseDetectorIter();
        }

    }


    detected_objects.clear();
    // Concat results into global results
    detected_objects.insert(
        detected_objects.end(),
        std::make_move_iterator(new_detected_objects.begin()),
        std::make_move_iterator(new_detected_objects.end())
    );


    return detected_objects.size();

}