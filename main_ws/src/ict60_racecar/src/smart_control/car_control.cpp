#include "car_control.h"


CarControl::CarControl()
{

    readConfig();

    steer_publisher = node_obj1.advertise<std_msgs::Float32>(config.getTeamName() + "_steerAngle", 1);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>(config.getTeamName() + "_speed", 1);

    publishSignal(0,0);
    last_signal_publish_time_point = Timer::getCurrentTime();

}

CarControl::~CarControl() {}

void CarControl::readConfig() {

    // Control Signal
    MAX_SPEED = config.get<float>("max_speed");
    MAX_ANGLE = config.get<float>("max_angle");
    signal_publish_interval = config.get<long int>("control_signal_publish_interval");

    line_diff_to_angle_coeff = config.get<float>("line_diff_to_angle_coeff");
    delta_to_angle_coeff = config.get<float>("delta_to_angle_coeff");
    middle_interested_point_pos = config.get<float>("middle_interested_point_pos");
    min_num_of_middle_points = config.get<int>("min_num_of_middle_points");
    min_traffic_sign_bound_area = config.get<int>("min_traffic_sign_bound_area");
    traffic_sign_valid_duration = config.get<int>("traffic_sign_valid_duration");
    speed_on_preparing_to_turn_trafficsign = config.get<float>("speed_on_preparing_to_turn_trafficsign");
    lane_area_to_turn = config.get<int>("lane_area_to_turn");
    turning_angle_on_trafficsign = config.get<float>("turning_angle_on_trafficsign");
    speed_on_turning_trafficsign = config.get<float>("speed_on_turning_trafficsign");
    turning_duration_trafficsign = config.get<int>("turning_duration_trafficsign");

   
}


void CarControl::publishSignal(float speed_data, float angle_data) {

    if (Timer::calcTimePassed(last_signal_publish_time_point) > signal_publish_interval) {
        std_msgs::Float32 angle;
        std_msgs::Float32 speed;

        angle.data = angle_data;
        speed.data = speed_data;

        // Fit to value ranges
        if (angle_data > MAX_ANGLE) angle_data = MAX_ANGLE;
        if (angle_data < -MAX_ANGLE) angle_data = -MAX_ANGLE;
        if (speed_data > MAX_SPEED) speed_data = MAX_SPEED;
        if (speed_data < MIN_SPEED) speed_data = MIN_SPEED;

        // Publish message
        if (debug_flag) {
            ROS_INFO_STREAM("SPEED: " << speed_data);
            ROS_INFO_STREAM("ANGLE: " << angle_data);
        }

        steer_publisher.publish(angle);
        speed_publisher.publish(speed);

        last_signal_publish_time_point = Timer::getCurrentTime();
    }
    
}


void CarControl::driverCar(float speed_data, float angle_data) {
    publishSignal(speed_data, angle_data);
}


void CarControl::driverCar(Road & road, const std::vector<TrafficSign> & traffic_signs) {

    //  STEP 1: FIND THE CAR POSITION

    // Do nothing when we find a bad result from lane detector
    if (road.middle_points.size() < min_num_of_middle_points) {
        return;
    }

    // Sort the middle points asc based on y
    std::sort(std::begin(road.middle_points), std::end(road.middle_points),
            [] (const cv::Point& lhs, const cv::Point& rhs) {
        return lhs.y < rhs.y;
    });

    // Choose an interested point (point having y = 60% ymax)
    int index_of_interested_point = static_cast<int>(road.middle_points.size() * middle_interested_point_pos);


    cv::Point middle_point = road.middle_points[index_of_interested_point];
    
    if (debug_flag) {
        std::cout << middle_point << std::endl;
    }
    
    cv::Point center_point = cv::Point(Road::road_center_line_x, middle_point.y);


    //  STEP 2: FIND THE "QUALITY" OF LANE DETECTION

    // // The total difference of x of 2 continuous point in the lane line
    // // Unit: pixel
    // double total_of_difference_left = 0;
    // double total_of_difference_right = 0;

    // // The average difference of x of 2 continuous point in the lane line
    // // Unit: pixel
    // double average_difference_left = 0;
    // double average_difference_right = 0;
    

    // for (size_t i = 1; i < road.left_points.size(); ++i) {
    //     total_of_difference_left += abs(road.left_points[i].x - road.left_points[i-1].x);
    // }
    // for (size_t i = 1; i < road.right_points.size(); ++i) {
    //     total_of_difference_right += abs(road.right_points[i].x - road.right_points[i-1].x);
    // }

    // // Calculate the average difference
    // average_difference_left = total_of_difference_left / road.left_points.size();
    // average_difference_right = total_of_difference_right / road.right_points.size();


    // if (debug_flag) ROS_INFO_STREAM("average_difference_left: " << average_difference_left);
    // if (debug_flag) ROS_INFO_STREAM("average_difference_right: " << average_difference_right);

    //  STEP 3: FIND THE BASE CONTROLLING PARAMS ( BASED ON LANE LINES )
    float speed_data = MAX_SPEED;
    float delta = center_point.x - middle_point.x;


    // line_diff_to_angle_coeff
    float line_diff = 0;
    if (index_of_interested_point > 12) {
        float diff1 = road.middle_points[index_of_interested_point].x -  road.middle_points[index_of_interested_point - 3].x;
        float diff2 = road.middle_points[index_of_interested_point - 3].x -  road.middle_points[index_of_interested_point - 6].x;
        if (abs(diff1 - diff2) < 6) {
            line_diff = diff1;
        }
        std::cout << "diff1: " << diff1 << std::endl;
        std::cout << "diff2: " << diff2 << std::endl;
    }

    float angle_data = delta * delta_to_angle_coeff + line_diff * line_diff_to_angle_coeff;

    speed_data -= abs(line_diff);


    //  STEP 4: ADJUST CONTROLLING PARAMS USING TRAFFIC SIGN DETECTOR
    if (!traffic_signs.empty() && !is_turning) {

        if (debug_flag) ROS_INFO_STREAM("TRAFFIC SIGN DETECTED!: " << "Number: " << traffic_signs.size());

        for (int i = 0; i < traffic_signs.size(); ++i) {
            std::cout << traffic_signs[i].id << " : " << traffic_signs[i].rect << std::endl;
        }


        if (traffic_signs[0].rect.area() > min_traffic_sign_bound_area) {

            prepare_to_turn = true;
            last_sign_id = traffic_signs[0].id;
            last_sign_time_point = Timer::getCurrentTime();

        }
    }


    // Giảm tốc khi đến khúc cua
    if (prepare_to_turn && Timer::calcTimePassed(last_sign_time_point) < traffic_sign_valid_duration) {
        speed_data = speed_on_preparing_to_turn_trafficsign;
    }

    // Khi nhận thấy đang có tín hiệu rẽ,
    // và diện tích đường mở rộng (đã đến ngã ba, ngã tư) thì thực hiện rẽ
    if (prepare_to_turn && road.lane_area > lane_area_to_turn && Timer::calcTimePassed(last_sign_time_point) < traffic_sign_valid_duration) {
        prepare_to_turn = false;
        if (debug_flag) ROS_INFO_STREAM("TURNING: " << last_sign_id);

        if (last_sign_id == TrafficSign::SignType::TURN_LEFT) {
            turning_coeff = -turning_angle_on_trafficsign;
        } else if (last_sign_id == TrafficSign::SignType::TURN_RIGHT) {
            turning_coeff = +turning_angle_on_trafficsign;
        }
        
        speed_data = speed_on_turning_trafficsign;
        turning_time_point = std::chrono::system_clock::now();
        is_turning = true;
    }


    if (debug_flag) {
        ROS_INFO_STREAM("turning_coeff: " << turning_coeff);
    }

    if (Timer::calcTimePassed(turning_time_point) > turning_duration_trafficsign) {
        turning_coeff = 0;
        is_turning = false;
    }

    if (is_turning) {
        speed_data = speed_on_turning_trafficsign;
    }
    


    // STEP 5: FIND AND AVOID OBSTACLE
    
    // Print line diff
    if (debug_flag) {
        ObstacleDetector::printLineDiff(road.middle_points);
    }
    

    // // Calculate the diff b/w current time and the last obstacle time
    // // If the time is out of obstacle avoiding range, reset obstacle_avoid_coeff
    // if (is_turning == true && Timer::calcTimePassed(obstacle_avoiding_time_point) > 1000) {
    //     obstacle_avoid_coeff = 0;
    //     ++success_turning_times;
    // }

    // if (debug_flag) {
    //     std::cout << "LAST OBSTACLE TIME: " << Timer::calcTimePassed(obstacle_avoiding_time_point) << std::endl;
    //     std::cout << "OBSTACLE COEFF: " << obstacle_avoid_coeff << std::endl;
    // }
    

    // Find the obstacle and adjust obstacle_avoid_coeff
    // for (int i = road.middle_points.size()-5; i >=  10; --i) {
    //     int diff = road.middle_points[i+3].x - road.middle_points[i].x;
    //     int distance_to_obstacle;

    //     if (abs(diff) > 5 and abs(diff) < 10) {
    //         distance_to_obstacle = Road::road_center_line_x - road.middle_points[i].y;
            
    //         if (debug_flag) {
    //             std::cout << "OBSTACLE DISTANCE: " << distance_to_obstacle << std::endl;
    //         }

    //         obstacle_avoiding_time_point = std::chrono::high_resolution_clock::now();
        
    //         if (diff > 0) {
    //             obstacle_avoid_coeff = -5;
    //             break;
    //         } else if (diff < 0) {
    //             obstacle_avoid_coeff = +5;
    //             break;
    //         }
        
    //     }

    // }
    

    angle_data += obstacle_avoid_coeff + turning_coeff;

    if (turning_coeff != 0) {
        angle_data = turning_coeff;
    }

    if (debug_flag) ROS_INFO_STREAM("lane_area: " << road.lane_area);


    // STEP 6: FINAL ADJUSTMENT AND PUBLISH

    // Filter the angle (remove small angle)
    if (abs(angle_data) < 1) {
        angle_data = 0;
    }


    publishSignal(speed_data, angle_data);

}
