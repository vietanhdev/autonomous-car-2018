#include "lane_detector.h"

void LaneDetector::initConfig() {

    // ** Floodfill
    // floodfill_lo = cv::Scalar(12,100,200);
    // floodfill_hi = cv::Scalar(12,100,200);
    floodfill_lo = cv::Scalar(4,30,100);
    floodfill_hi = cv::Scalar(4,30,100);

    floodfill_points.push_back(cv::Point(150, 230));
    floodfill_points.push_back(cv::Point(160, 230));
    floodfill_points.push_back(cv::Point(170, 230));

}

// ** Initialize perspective transform matrices
void LaneDetector::initPerspectiveTransform() {

    // TRANSFORM
    std::vector<cv::Point2f> corners_source(4);
    std::vector<cv::Point2f> corners_trans(4);

    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input 
    corners_source[0] =  cv::Point2f( 0, 100 );
    corners_source[1] =  cv::Point2f( 319, 100 );
    corners_source[2] =  cv::Point2f( 319, 239 );
    corners_source[3] =  cv::Point2f( 0, 239 );


    // The 4 points where the mapping is to be done , from top-left in clockwise order
    corners_trans[0] =  cv::Point2f( 0, 0 );
    corners_trans[1] =  cv::Point2f( 319, 0 );
    corners_trans[2] =  cv::Point2f( 200, 150 );
    corners_trans[3] =  cv::Point2f( 120, 150 );

    getPerspectiveMatrix(corners_source, corners_trans);

    perspective_img_size = cv::Size(320, 150);

    cv::Mat tmp(240, 320, CV_8UC1, cv::Scalar(255));
    perspectiveTransform(tmp, interested_area);

}

// ** Constructor
LaneDetector::LaneDetector() {

    initConfig();
    initPerspectiveTransform();
}


cv::Point LaneDetector::getNullPoint() {
    return cv::Point(-1, -1);
}

/*--- Floodfill ---*/
void LaneDetector::laneFloodFill(const cv::Mat & img, cv::Mat & dst, cv::Point start_point) {

    int ffillMode = 2; // 2: gradient fill, 1: Fixed Range floodfill

    int connectivity = 4; // or 8?
    int newMaskVal = 255;
    int flags = connectivity + (newMaskVal << 8) +
            (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);

    // Working in HSV color space
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    int area;
    cv::Point seed = start_point;
    cv::Scalar newVal(255);

    // Flood fill
    cv::Mat result;
    cv::Mat mask = cv::Mat::zeros(cv::Size(img.cols+2,img.rows+2), CV_8UC1);
    cv::cvtColor(img, result, cv::COLOR_BGR2GRAY);
    area = cv::floodFill(result, mask, seed, newVal, 0, floodfill_lo, floodfill_hi, flags);

    dst = mask;
}

void LaneDetector::laneFloodFillPoints(const cv::Mat & input, cv::Mat & mask) {
    
    if (floodfill_points.empty()) return;

    cv::Mat img  = input.clone();

    // Floodfill from the first point
    cv::Mat m1;
    laneFloodFill(img, m1, floodfill_points[0]);
    mask = m1.clone();

    // Floodfill from reamining points
    for (size_t i = 0; i < floodfill_points.size(); ++i) {
        laneFloodFill(img, m1, floodfill_points[i]);
        mask |= m1;
    }

}


void LaneDetector::doCannyEdges(const cv::Mat & img, cv::Mat & mask) {
    blur( img, mask, cv::Size(3,3) );
    int low_threshold = 50;
    int high_threshold = 200;
    cv::Canny(img, mask, low_threshold, high_threshold,  3);
}

int LaneDetector::getPerspectiveMatrix(const std::vector<cv::Point2f> corners_source,
                                    const std::vector<cv::Point2f> corners_trans) {
    if (corners_source.size() != 4 || corners_trans.size() != 4) {
        std::cout<< "error in GetPerspectiveMatrix" <<std::endl;
        return false;
    }//if
    perspective_matrix_ = cv::getPerspectiveTransform(corners_source, corners_trans);
    inverse_perspective_matrix_ = cv::getPerspectiveTransform(corners_trans, corners_source);
    return true;
}//GetPerspectiveMatrix

bool LaneDetector::findLaneMask(const cv::Mat & img, cv::Mat & mask) {

    // ** do Floodfill
    cv::Mat flood;
    laneFloodFillPoints(img, flood);

    // ** Separate the white area to distinct road and other things
    int dilate_size = 1;
    cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
        cv::Size( 2*dilate_size + 1, 2*dilate_size+1 ),
        cv::Point( dilate_size, dilate_size ) );
    dilate( flood, flood, dilate_element );

    int erosion_size = 5;
    cv::Mat erosion_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
        cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
        cv::Point( erosion_size, erosion_size ) );
    erode( flood, flood, erosion_element );

    dilate_size = 2;
    dilate_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
        cv::Size( 2*dilate_size + 1, 2*dilate_size+1 ),
        cv::Point( dilate_size, dilate_size ) );
    dilate( flood, flood, dilate_element );


    // ** Find the biggest area => road

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours( flood, contours, hierarchy, cv::RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

    if (contours.empty()) {
        return false;
    }

    mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

    int largest_area=0;
    int largest_contour_index=-1;

    // iterate through each contour. 
    for( int i = 0; i< contours.size(); i++ ) {
        double a = contourArea( contours[i],false);  //  Find the area of contour
        if( a > largest_area ){
            largest_area = a;
            largest_contour_index = i; //Store the index of largest contour
        }
    }


    lane_area = contourArea( contours[largest_contour_index], false);

    drawContours( mask, contours, largest_contour_index, cv::Scalar(255), CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.

    return true;

}

void LaneDetector::perspectiveTransform(const cv::Mat & src, cv::Mat & dst) {
    cv::warpPerspective(src, dst, perspective_matrix_, perspective_img_size);
}

void LaneDetector::removeCenterLaneLine(const cv::Mat & mask, cv::Mat output_mask) {

    cv::Mat inner_objects = mask.clone();
    floodFill(inner_objects, cv::Point(0, perspective_img_size.height-1), cv::Scalar(255));
    floodFill(inner_objects, cv::Point(perspective_img_size.width-1, perspective_img_size.height-1), cv::Scalar(255));

    cv::threshold (inner_objects, inner_objects, 1, 255, CV_THRESH_BINARY_INV);

    int dilate_size = 2;
    cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                    cv::Size( 2*dilate_size + 1, 2*dilate_size+1 ),
                    cv::Point( dilate_size, dilate_size ) );
    dilate( inner_objects, inner_objects, dilate_element );

    output_mask = (mask | inner_objects);

}

void LaneDetector::findEdgePoints(const cv::Mat & mask, size_t row, cv::Point & left_point, cv::Point & right_point) {

    // ** Find left point (this point lies on the left lane line)
    int col = 0;

    // Skip area not in interested area 
    while (interested_area.at<uchar>(cv::Point(col,row)) == 0
    ) {
        ++col;
        if (col >= perspective_img_size.width) break;
    }

    // Find a not-lane-line pixel
    while (col < perspective_img_size.width
        && mask.at<uchar>(cv::Point(col,row)) > 0
    ) {
        ++col;
        if (col >= perspective_img_size.width) break;
    }

    // Find left point
    while (col < perspective_img_size.width
        && mask.at<uchar>(cv::Point(col,row)) == 0
    ) {
        ++col;
        if (col >= perspective_img_size.width) break;
    }

    // Check if found a left point
    if (col < perspective_img_size.width) {
        left_point = cv::Point(col, row);
    } else {
        left_point = getNullPoint();
    }


    // ** Find right point (this point lies on the right lane line)
    col = perspective_img_size.width - 1;

    // Skip area not in interested area 
    while (interested_area.at<uchar>(cv::Point(col,row)) == 0
    ) {
        --col;
        if (col < 0) break;
    }

    // Find a not-lane-line pixel
    while (col >= 0
        && mask.at<uchar>(cv::Point(col,row)) > 0
    ) {
        --col;
        if (col < 0) break;
    }

    // Find right point
    while (col >= 0
        && mask.at<uchar>(cv::Point(col,row)) == 0
    ) {
        --col;
        if (col < 0) break;
    }

    // Check if found a right point
    if (col >= 0) {
        right_point = cv::Point(col, row);
    } else {
        right_point = getNullPoint();
    }


}


void LaneDetector::findLaneEdges(const cv::Mat & img, Road & road) {

    cv::Mat tmp;
    cvtColor(img, tmp, cv::COLOR_GRAY2BGR);

    cv::Point left, right;
    cv::Point middle;

    road.left_points.clear();
    road.right_points.clear();
    road.middle_points.clear();

    for (int i = 0; i < perspective_img_size.height - 1; i += 5) {
        findEdgePoints(img, i, left, right);

        if (left != getNullPoint()
            && right == getNullPoint()
        ) {
            right = cv::Point(left.x + Road::road_width, left.y);
        }

        if (right != getNullPoint()
            && left == getNullPoint()
        ) {
            left = cv::Point(right.x - Road::road_width, right.y);
        }

        if (right != getNullPoint() 
            && left != getNullPoint() 
        ) {
            middle = (left + right) / 2;

            if (left.x < 10  || left.y < 10 || right.x < 10 || right.y < 10) {
                std::cout << left << std::endl;
                std::cout << right << std::endl;
            }

            if (debug_show_image) {
                circle(tmp, middle, 1, cv::Scalar(255,0,0), 2);
            }

            road.left_points.push_back(left);
            road.right_points.push_back(right);
            road.middle_points.push_back(middle);

        }

        if (debug_show_image) {
            circle(tmp, left, 1, cv::Scalar(0,255,0), 2);
            circle(tmp, right, 1, cv::Scalar(0,0,255), 2);
        }
    }

    
    if (debug_show_image) {
        cv::imshow("edge points", tmp);
        cv::waitKey(1);
    }  

}


void LaneDetector::findLanes(const cv::Mat & input, Road & road) {
    
    cv::Mat img = input.clone();

    cv::Mat lane_mask;

    cv::Mat canny_edges;
    doCannyEdges(img, canny_edges);

    cv::Mat canny_edges_bgr;
    cv::cvtColor(canny_edges, canny_edges_bgr, cv::COLOR_GRAY2BGR);

    img = img | canny_edges_bgr;

    if (debug_show_image) {
        cv::imshow("img + canny_edges", img);
        cv::waitKey(1);
    }  
    

    bool lane_mask_result = findLaneMask(img, lane_mask);

    // TODO: FIX this
    // Dont just return
    if (!lane_mask_result) return;

    if (debug_show_image) {
        cv::imshow("lane_mask", lane_mask);
        cv::waitKey(1);
    }  

    perspectiveTransform(lane_mask, lane_mask);

    removeCenterLaneLine(lane_mask, lane_mask);

    if (debug_show_image) {
        cv::imshow("lane_mask > perspective transform", lane_mask);
        cv::waitKey(1);
    }  

    findLaneEdges(lane_mask, road);

    road.lane_area = lane_area;

    if (debug_show_image) {
        cv::imshow("canny_edges", canny_edges);
        cv::waitKey(1);
    } 


}
