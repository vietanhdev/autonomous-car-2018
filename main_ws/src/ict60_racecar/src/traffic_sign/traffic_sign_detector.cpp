#include "traffic_sign_detector.h"

// ==================================================
// ********** INITIALIZE **********
// ==================================================
TrafficSignDetector::TrafficSignDetector(){

    debug_flag = true;
    config_trafficsign = Config("config_trafficsign.yaml");

    std::cout << config_trafficsign.get<std::string>("tsd_version") << std::endl;

    std::string model_file = ros::package::getPath(config.getROSPackage()) + config.get<std::string>("traffic_sign_detector_svmfile");
    model = cv::Algorithm::load<cv::ml::SVM>(model_file);

    low_HSV = cv::Scalar(90, 110, 110);
    high_HSV = cv::Scalar(104, 255, 255);

    size = 32;
    eps_diff = 1.5;

    // For filtering bouding rects, compare high with min_accepted_size, ratio = high/width
    min_accepted_size = 10;
    min_accepted_ratio = 0.9;
    max_accepted_ratio = 1.5;

    // Number of labeled bouding rects in previous frames is stored in 'record'
    // If there are enough similarity labeled bouding rects in 'record', we can label the current rects as them
    num_prev_check = 5;
    num_certainty = 3;

    // Init HOG Descriptor config
    hog = cv::HOGDescriptor(
        cv::Size(size,size),    //winSize
        cv::Size(8, 8),         //blocksize
        cv::Size(4, 4),         //blockStride,
        cv::Size(8, 8),         //cellSize,
        9,                      //nbins,
        1,                      //derivAper,
        -1,                     //winSigma,
        0,                      //histogramNormType,
        0.2,                    //L2HysThresh,
        1,                      //gamma correction,
        64,                     //nlevels=64
        1                       //_signedGradient = true
    );
};


// ==================================================
// ********** HELPER **********
// ==================================================

void TrafficSignDetector::createHOG(cv::HOGDescriptor &hog, std::vector<std::vector<float>> &HOG, std::vector<cv::Mat> &cells){
    for(size_t i=0; i<cells.size(); i++){
        std::vector<float> descriptors;
        hog.compute(cells[i], descriptors);
        HOG.push_back(descriptors);
    }
}

void TrafficSignDetector::cvtVector2Matrix(std::vector<std::vector<float>> &HOG, cv::Mat &mat){
    int descriptor_size = HOG[0].size();

    for(size_t i=0; i<HOG.size(); i++){
        for(size_t j=0; j<descriptor_size; j++){
            mat.at<float>(i, j) = HOG[i][j];
        }
    }
}


// ==================================================
// ********** SVM **********
// ==================================================

void TrafficSignDetector::svmPredict(cv::Ptr<cv::ml::SVM> svm, cv::Mat &response, cv::Mat &mat){
    svm->predict(mat, response);
}


// ==================================================
// ********** CLASSIFY **********
// ==================================================

int TrafficSignDetector::classifySVM(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &model, cv::Mat &img){

	std::vector<cv::Mat> cells;
    cvtColor(img, img, cv::COLOR_BGR2GRAY);
	cells.push_back(img);

	std::vector<std::vector<float>> HOG;
    createHOG(hog, HOG, cells);

    cv::Mat mat(HOG.size(), HOG[0].size(), CV_32FC1);

    cvtVector2Matrix(HOG, mat);

    cv::Mat response;
    svmPredict(model, response, mat);

    return (int)(response.at<float>(0, 0));
}


// ==================================================
// ********** TRAFFIC SIGN DETECTOR **********
// ==================================================

void TrafficSignDetector::BrightnessAndContrastAuto(cv::Mat src, cv::Mat &dst, bool clipHistPercent=false){
    int histSize = 256;
    float alpha, beta;
    double minGray = 0, maxGray = 0;

    cv::Mat gray;

    cvtColor(src, gray, CV_BGR2GRAY);

    if (clipHistPercent == true) {
        // keep full available range
        cv::minMaxLoc(gray, &minGray, &maxGray);
    } else {
        cv::Mat hist; //the grayscale histogram

        float range[] = { 0, 256 };
        const float* histRange = { range };
        bool uniform = true;
        bool accumulate = false;
        calcHist(&gray, 1, 0, cv::Mat (), hist, 1, &histSize, &histRange, uniform, accumulate);

        // calculate cumulative distribution from the histogram
        std::vector<float> accumulator(histSize);
        accumulator[0] = hist.at<float>(0);
        for (int i = 1; i < histSize; i++)
        {
            accumulator[i] = accumulator[i - 1] + hist.at<float>(i);
        }

        // locate points that cuts at required value
        float max = accumulator.back();
        clipHistPercent *= (max / 100.0); //make percent as absolute
        clipHistPercent /= 2.0; // left and right wings
        // locate left cut
        minGray = 0;
        while (accumulator[minGray] < clipHistPercent)
            minGray++;

        // locate right cut
        maxGray = histSize - 1;
        while (accumulator[maxGray] >= (max - clipHistPercent))
            maxGray--;
    }

    float input_range = maxGray - minGray;
    float output_range = 255;
    alpha = output_range/input_range;

    beta = -minGray * alpha;

    src.convertTo(dst, -1, alpha, beta);
}

void TrafficSignDetector::inRangeHSV(cv::Mat &bin_img){
	cv::Mat img_HSV, img_threshold;

	// Convert color from BGR to HSV color space
	cvtColor(img, img_HSV, cv::COLOR_BGR2HSV);

	// Mark out all points in range, return binary image
	inRange(img_HSV, low_HSV, high_HSV, bin_img);

    if(debug_flag == true){
        imshow("inRangeHSV", bin_img);
        // cv::waitKey(1);
    }
}

void TrafficSignDetector::boundRectBinImg(cv::Mat bin_img, std::vector<cv::Rect> &bound_rects){
	int eps_diff = 0.01;

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	findContours(bin_img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	std::vector<std::vector<cv::Point>> contours_poly( contours.size() );
	cv::Rect rect;

	for(size_t i=0; i<contours.size(); i++){
		int contour_area = contourArea(contours[i]);
		approxPolyDP(contours[i], contours_poly[i], 3, true);
		rect = boundingRect(contours_poly[i]);

		bound_rects.push_back(rect);
	}
}

void TrafficSignDetector::boundRectByColor(std::vector<cv::Rect> &bound_rects){
	// Apply threshold for BGR image
	cv::Mat bin_img;
	inRangeHSV(bin_img);

	// Get all bound rects
	boundRectBinImg(bin_img, bound_rects);
}

void TrafficSignDetector::mergeRects(std::vector<cv::Rect> &bound_rects){
	int findIntersection;

	do{
		findIntersection = false;

		for(auto it_1 = bound_rects.begin(); it_1 != bound_rects.end(); it_1++){
			for(auto it_2 =it_1+1; it_2 != bound_rects.end();){
				if( ((*it_1) & (*it_2)).area() > 0 ){
					findIntersection = true;
					*it_1 = ((*it_1) | (*it_2));
					bound_rects.erase(it_2);
				} else {
					it_2++;
				}
			}
		}
	} while(findIntersection == true);
}

void TrafficSignDetector::extendRect(cv::Rect &rect, int extend_dist){
	int tl_x = (rect.tl().x - extend_dist > 0) ? rect.tl().x - extend_dist : rect.tl().x;
	int tl_y = (rect.tl().y - extend_dist > 0) ? rect.tl().y - extend_dist : rect.tl().y;
	int br_x = (rect.br().x + extend_dist < width) ? rect.br().x + extend_dist : rect.br().x;
	int br_y = (rect.br().y + extend_dist < height) ? rect.br().y + extend_dist : rect.br().y;
	rect.x = tl_x;
	rect.y = tl_y;
	rect.width = br_x - tl_x;
	rect.height = br_y - tl_y;
}

bool TrafficSignDetector::checkSimilarityRect(cv::Rect A, cv::Rect B){
	float x = (float)(A|B).area();
	float y = (float)A.area() + (float)B.area() - (float)(A&B).area();
	float ratio = x / y;
	if( ratio < eps_diff ){
		return true;
	}
	return false;
}

void TrafficSignDetector::classifyRect(){

	for(int i=0; i<record.curr_rects.size(); i++){

		int classified = false;
		int count = 0;

		for(size_t j=0; j<record.prev_rects.size(); j++){
			if(checkSimilarityRect(record.curr_rects[i].rect, record.prev_rects[j].rect) ==  true){
				// found the similar classified rect in prev_rects, we classify curr rect base on prev label
				count++;
				if(count >= num_certainty){
					record.curr_rects[i].id = record.prev_rects[j].id;
					classified = true;
					break;
				}
			}
		}

		if(classified == false){
			// the curr_rects is not appear in prev_rects, so we have to use classifySVM
			cv::Mat roi_img = img(record.curr_rects[i].rect);
			resize(roi_img, roi_img, cv::Size(size, size));
			int id = classifySVM(hog, model, roi_img);
			record.curr_rects[i].id = id;
		}

	}
}

void TrafficSignDetector::recognize(const cv::Mat & input, std::vector<TrafficSign> &traffic_signs){

    cv::Mat frame = input.clone();

	// Preprocessing, auto adjust brightness and contrast image
	BrightnessAndContrastAuto(frame, img, 1);
    width = img.cols;
    height = img.rows;

	// Clear old curr_rects to record new one
	record.curr_rects.clear();

	std::vector<cv::Rect> bound_rects;

	// Detect all bound_rects by color and contour area
    boundRectByColor(bound_rects);

    // Merge all rects have intersection greater than 0
    mergeRects(bound_rects);

    // Merge all neighbor rects by extending their size in 4 directions, then merge again by mergeRects function
    for(size_t i=0; i<bound_rects.size(); i++){
        extendRect(bound_rects[i], 1);
    }
    mergeRects(bound_rects);

    // Filter bound_rects by height and width
    for(size_t i=0; i<bound_rects.size(); i++){
    	float width = bound_rects[i].width;
    	float height = bound_rects[i].height;
    	float ratio = height/width;

    	if(height > min_accepted_size 
            && ratio >= min_accepted_ratio && ratio < max_accepted_ratio){
    		record.curr_rects.push_back(TrafficSign(0, bound_rects[i]));
    	}
    }

	classifyRect();

	if(record.count_rects.size() >= num_prev_check){
		record.prev_rects.erase(record.prev_rects.begin(), record.prev_rects.begin() + record.count_rects[0]);
		record.count_rects.erase(record.count_rects.begin());
	}

	for(int i=0; i<record.curr_rects.size(); i++){
		record.prev_rects.push_back(record.curr_rects[i]);
	}
	record.count_rects.push_back(record.curr_rects.size());

    // Return value to traffic_signs
    traffic_signs = record.curr_rects;

    if(debug_flag == true){
        for(size_t i=0; i<traffic_signs.size(); i++){
            if(traffic_signs[i].id != 0){
                int x = traffic_signs[i].rect.tl().x;
                int y = traffic_signs[i].rect.tl().y;
                std::string text = traffic_signs[i].id == 1? "left":"right";
                putText(img, text, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,255), 2.0);
                std::cout << text << " at [" << x << ", " << y << "]" << std::endl;
            }
            rectangle(img, traffic_signs[i].rect.tl(), traffic_signs[i].rect.br(), CV_RGB(255,0,255), 1, 8, 0);
        }
        imshow("traffic sign detection", img);
        // cv::waitKey(1);
    }
}