#include "traffic_sign_detector.h"


using namespace cv;
using namespace std;
using namespace cv::ml;

TrafficSignDetector::TrafficSignDetector() {

    color_file = ros::package::getPath("ict60_racecar") + std::string("/data/blue.color");
    svm_file = ros::package::getPath("ict60_racecar") + std::string("/data/svm_an_linear.xml");


    cout << "color_file: " << color_file << endl;
    cout << "svm_file: " << svm_file << endl;

    // Init HOG Descriptor config
    hog = cv::HOGDescriptor(
        Size(size,size),    //winSize
        Size(8, 8),          //blocksize
        Size(4, 4),          //blockStride,
        Size(8, 8),          //cellSize,
        9,                  //nbins,
        1,                  //derivAper,
        -1,                 //winSigma,
        0,                  //histogramNormType,
        0.2,                //L2HysThresh,
        1,                  //gamma correction,
        64,                 //nlevels=64
        1
    );                 //_signedGradient = true 

    readColorFile();

    cout << "Done loading color file." << endl;

    model = Algorithm::load<SVM>(svm_file);
    cout << "Done loading detectors." << endl;
    getSVMParams(model);
    std::cout << "var_count = " << model->getVarCount() << endl;

}

// =====================================================
// ******* HELPER ********
// ======================================================

void TrafficSignDetector::createHOG(HOGDescriptor &hog, vector<vector<float>> &HOG, vector<Mat> &cells){
    for(int i=0; i<cells.size(); i++) {
        vector<float> descriptors;
        hog.compute(cells[i], descriptors);
        HOG.push_back(descriptors);
    }
}

void TrafficSignDetector::cvtVector2Matrix(vector<vector<float>> &HOG, Mat &mat){
    int descriptor_size = HOG[0].size();

    for(int i=0; i<HOG.size(); i++){
        for(int j=0; j<descriptor_size; j++){
            mat.at<float>(i, j) = HOG[i][j];
        }
    }
}


// =====================================================
// ******* SVM ********
// ======================================================

Ptr<SVM> TrafficSignDetector::svmInit(float C, float gamma){
    Ptr<SVM> svm = SVM::create();
    svm->setGamma(gamma);
    svm->setC(C);
    svm->setKernel(SVM::LINEAR);
    svm->setType(SVM::C_SVC);

    return svm;
}

void TrafficSignDetector::getSVMParams(SVM *svm){
    cout << "Kernel type     : " << svm->getKernelType() << endl;
    cout << "Type            : " << svm->getType() << endl;
    cout << "C               : " << svm->getC() << endl;
    cout << "Degree          : " << svm->getDegree() << endl;
    cout << "Nu              : " << svm->getNu() << endl;
    cout << "Gamma           : " << svm->getGamma() << endl;
}

void TrafficSignDetector::svmPredict(Ptr<SVM> svm, Mat &test_response, Mat &test_mat ){
    svm->predict(test_mat, test_response);
}


// =====================================================
// ******* DETECT ********
// ======================================================


void TrafficSignDetector::inRangeHSV(Mat &img, Mat &bin_img, cv::Scalar low_HSV, cv::Scalar high_HSV){
	
	Mat img_HSV, img_threshold;

	// Convert color from BGR to HSV color space
	cvtColor(img, img_HSV, COLOR_BGR2HSV);

	// Mark out all points in range, return binary image
	inRange(img_HSV, low_HSV, high_HSV, bin_img);
}

void TrafficSignDetector::boundRectBinImg(Mat &img, vector<Rect> &bound_rects){
	int eps_diff = 0.01;

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<vector<Point>> contours_poly( contours.size() );
	Rect rect;

	for(int i=0; i<contours.size(); i++){

		int contour_area = contourArea(contours[i]);
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		rect = boundingRect(Mat(contours_poly[i]));

		bound_rects.push_back(rect);

	}
}

void TrafficSignDetector::boundRectByColor(Mat &img, vector<Rect> &bound_rects, cv::Scalar low_HSV, cv::Scalar high_HSV){
	// Apply threshold for BGR image
	Mat bin_img;
	inRangeHSV(img, bin_img, low_HSV, high_HSV);
	// imshow("bin_img", bin_img);

	// Get all bound rects
	boundRectBinImg(bin_img, bound_rects);
}

void TrafficSignDetector::mergeRects(vector<Rect> &bound_rects){
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

void TrafficSignDetector::extendRect(Rect &rect, int extend_dist, int limit_br_x, int limit_br_y){
	int tl_x = (rect.tl().x - extend_dist > 0) ? rect.tl().x - extend_dist : rect.tl().x;
	int tl_y = (rect.tl().y - extend_dist > 0) ? rect.tl().y - extend_dist : rect.tl().y;
	int br_x = (rect.br().x + extend_dist < limit_br_x) ? rect.br().x + extend_dist : rect.br().x;
	int br_y = (rect.br().y + extend_dist < limit_br_y) ? rect.br().y + extend_dist : rect.br().y;
	rect.x = tl_x;
	rect.y = tl_y;
	rect.width = br_x - tl_x;
	rect.height = br_y - tl_y;
}

bool TrafficSignDetector::checkSimilarityRect(Rect A, Rect B, float eps_diff){
	float x = (float)(A|B).area();
	float y = (float)A.area() + (float)B.area() - (float)(A&B).area();
	float ratio = x / y;
	if( ratio < eps_diff ){
		return true;
	}
	return false;
}


// =====================================================
// ******* CLASSIFY ********
// ======================================================

void TrafficSignDetector::classifyRect(Mat &img, 
	vector<Rect> &curr_rects, vector<int> &curr_labels, 
	vector<Rect> &prev_rects, vector<int> &prev_labels, 
	int size, float eps_diff){

	for(int i=0; i<curr_rects.size(); i++){

		int classified = false;
		int count = 0;

		for(int j=0; j<prev_rects.size(); j++){
			if(checkSimilarityRect(curr_rects[i], prev_rects[j], eps_diff) ==  true){
				// found the similar classified rect in prev_rects, we classify curr rect base on prev_labels
				count++;
				if(count > 2){
					curr_labels.push_back(prev_labels[i]);
					classified = true;
					break;
				}
			}
		}

		if(classified == false){
			// the curr_rects is not appear in prev_rects, so we have to use classifySVM
			Mat roi_img = img(curr_rects[i]);
			resize(roi_img, roi_img, Size(size, size));
			int label = classifySVM(roi_img);
			curr_labels.push_back(label);
		}

	}
}

void TrafficSignDetector::trafficDetect(Mat &img,
	vector<Rect> &curr_rects, vector<int> &curr_labels,
	vector<Rect> &prev_rects, vector<int> &prev_labels,
	int size, float eps_diff,
	cv::Scalar low_HSV, cv::Scalar high_HSV){

	// Detect all curr_rects by color and contour area
    boundRectByColor(img, curr_rects, low_HSV, high_HSV);

    // Merge all rects have intersection greater than 0
    mergeRects(curr_rects);

    // Merge all neighbor rects by extending their size in 4 directions, then merge again by mergeRects function
    for(int i=0; i<curr_rects.size(); i++){
        extendRect(curr_rects[i], 1, img.cols, img.rows);
    }
    mergeRects(curr_rects);

    // filter curr_rects by height and width
    for(auto it = curr_rects.begin(); it != curr_rects.end();){
	   	float width = (*it).width;
	    float height = (*it).height;
	    float ratio = (float)(height/width);

	    if(height < size*0.8 || ratio <=0.9 || ratio > 1.5){
	    	curr_rects.erase(it);
	    } else {
	    	it++;
	    }
    }

	classifyRect(img, 
				curr_rects, curr_labels, 
				prev_rects, prev_labels, 
				size, eps_diff);

	for(int i=0; i<curr_rects.size(); i++){
		prev_rects.push_back(curr_rects[i]);
		prev_labels.push_back(curr_labels[i]);
	}
}


int TrafficSignDetector::classifySVM(Mat &img){
    
	vector<Mat>	cells;
    cvtColor(img, img, COLOR_BGR2GRAY);
	cells.push_back(img);

	vector<vector<float>> HOG;
    createHOG(hog, HOG, cells);

    Mat mat(HOG.size(), HOG[0].size(), CV_32FC1);

    cvtVector2Matrix(HOG, mat);

    Mat response;
    svmPredict(model, response, mat);

    return (int)(response.at<float>(0, 0));
}


void TrafficSignDetector::recognize(const cv::Mat & input, std::vector<TrafficSign> & classification_results) {

    cv::Mat frame = input.clone();

    // Clear the result first
    classification_results.clear();

    vector<Scalar> color{
        Scalar(255, 0, 0),
        Scalar(0, 255, 0),
        Scalar(0, 0, 255),
    };

    ////////// USE THIS FUNCTION IN YOUR CODE TO DETECT AND CLASSIY TRAFFIC SIGN //////////

    vector<Rect> curr_rects;
    vector<int> curr_labels;

    // I also classify base on width and height of rect, you can modify those number in this function
    trafficDetect(frame, 
        curr_rects, curr_labels,
        prev_rects, prev_labels,
        size, eps_diff,
        color_ranges[0].begin, color_ranges[0].end);

    // hist enque the curr_rects size
    hist.push_back(curr_rects.size());

    // save only 5 previous frames
    if(hist.size() > 5){
        // "deque" prev_rects and prev_labels by delete hist[0] number of first elements
        prev_rects.erase(prev_rects.begin(), prev_rects.begin() + hist[0]);
        prev_labels.erase(prev_labels.begin(), prev_labels.begin() + hist[0]);

        // deque first element in hist
        hist.erase(hist.begin());
    }

    // draw bounding rect and color based on label
    for(int i=0; i<curr_rects.size(); i++){
        if(curr_labels[i] != 0){
            rectangle(frame, curr_rects[i].tl(), curr_rects[i].br(), color[curr_labels[i]], 1, 8, 0);

            classification_results.push_back(TrafficSign(curr_labels[i], curr_rects[i]));
        }
    }

    resize(frame, frame, Size(frame.cols*2, frame.rows*2));
    imshow("Debug Traffic Sign", frame);
    cv::waitKey(1);

}


void TrafficSignDetector::readColorFile() {
    // Load color file
    std::string line;
    std::ifstream color_file_stream (color_file);
    if (!color_file_stream.is_open()) {
        std::cout << "Cannot open file: " << "blue.color" << std::endl;
        return;
    }

    color_ranges.clear();

    // Skip first 5 lines (comments)
    std::getline (color_file_stream, line);
    std::getline (color_file_stream, line);
    std::getline (color_file_stream, line);
    std::getline (color_file_stream, line);
    std::getline (color_file_stream, line);


    // HSV color range (from Scalar(h1, s1, v1) to Scalar(h2, s2, v2))
    double h1, s1, v1;
    double h2, s2, v2;
    
    while (color_file_stream) {
        std::getline (color_file_stream, line);
        if (line == "" || line == "END") break;
        std::stringstream line_stream(line);
        line_stream >> h1 >> s1 >> v1 >> h2 >> s2 >> v2;
        color_ranges.push_back(ColorRange(h1,s1,v1,h2,s2,v2));
    }
    
}