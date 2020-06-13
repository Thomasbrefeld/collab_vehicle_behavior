#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Includes for working with images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <collab_vehicle_behavior/CommanderConfig.h>

#include <vector>
#include <limits> //9e99 for infinity
#include <string>

#define CVWIN_PREVIEW_1 "Image Preview"
#define CVWIN_PREVIEW_2 "Mask Preview"
#define CVWIN_PREVIEW_3 "Mask Dialated Preview"

class CollabVehicleBehavior{
public:
    CollabVehicleBehavior();
    ~CollabVehicleBehavior();
    void scanCb(const sensor_msgs::LaserScan& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void configCallback(collab_vehicle_behavior::CommanderConfig &config, uint32_t level);
    void calculate_move(std::tuple<int, int, std::tuple<float, int>> objects);
    void stop();

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber scan_sub_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    dynamic_reconfigure::Server<collab_vehicle_behavior::CommanderConfig> server_;

    double hue_low_;
    double hue_high_;
    double sat_low_;
    double sat_high_;
    double lum_low_;
    double lum_high_;
    int dilation_size_;
    int largest_area_;

    int orange_loc = -360;

    int first_loop = 0;
    ros::Time last_time;
    std::vector<float> change_over;

    int max_speed = 2;
    float distance;
    float speed;
    float angle;
};

CollabVehicleBehavior::CollabVehicleBehavior() : nh_{"~"}, it_{nh_}{

    // Subscribe to the laser scan
    scan_sub_ = nh_.subscribe("/scan", 5, &CollabVehicleBehavior::scanCb, this);

    image_sub_ = it_.subscribe("/cam_pub/image_raw", 1, &CollabVehicleBehavior::imageCallback, this);

    server_.setCallback(boost::bind(&CollabVehicleBehavior::configCallback, this, _1, _2));

    // Publish on the twist command topic
    pub_ = nh_.advertise<geometry_msgs::Twist>("/prizm/twist_controller/twist_cmd", 10);
}

CollabVehicleBehavior::~CollabVehicleBehavior(){
    cv::destroyWindow(CVWIN_PREVIEW_1);
    cv::destroyWindow(CVWIN_PREVIEW_2);
    //cv::destroyWindow(CVWIN_PREVIEW_3);
}

void CollabVehicleBehavior::configCallback(collab_vehicle_behavior::CommanderConfig &config, uint32_t level){
    hue_low_ = config.hue_low;
    hue_high_ = config.hue_high;
    sat_low_ = config.sat_low;
    sat_high_ = config.sat_high;
    lum_low_ = config.lum_low;
    lum_high_ = config.lum_high;
    dilation_size_ = config.dilation_size;
    largest_area_ = config.largest_area;
}

void CollabVehicleBehavior::scanCb(const sensor_msgs::LaserScan& msg){
    int head = -1;
    int tail = -1;
    float last_range = 9e99;
    std::tuple<float,int> min (9e99,-1);
    std::vector<std::tuple<int, int, std::tuple<float, int>>> temp_objects;
    std::vector<std::tuple<int, int, std::tuple<float, int>>> objects;
    std::tuple<int, int, std::tuple<float, int>> temp;

    int is_object = false;
    int hold = 0;
    for (int x = 90; x < 270; ++x){
        if(msg.ranges[x] > .05 && msg.ranges[x] < 1.2){ //a range that is within the bounds of the lidar
            if (is_object){
                if(std::get<0>(min) > msg.ranges[x]){ //checks if the current range is the closest range
                    std::get<0>(min) = msg.ranges[x];
                    std::get<1>(min) = x;
                }
            }
            else{
                is_object = true;
                head = x;
            }
            tail = x;
        }
        else{
            if(is_object){
                std::get<0>(temp) = head;
                std::get<1>(temp) = tail;
                std::get<2>(temp) = min;
                temp_objects.push_back(temp);
                is_object = false;
                std::get<0>(min) = 9e99;
                std::get<1>(min) = -1;
                /*
                ++hold;
                if(hold == 1){
                    std::get<0>(temp) = head;
                    std::get<1>(temp) = tail;
                    std::get<2>(temp) = min;
                }
                else if(hold >= 10){
                    temp_objects.push_back(temp);
                    is_object = false;
                    std::get<0>(min) = 9e99;
                    std::get<1>(min) = -1;
                    hold = 0;
                }
                */
            }
        }
    }
    
    int t = objects.size();
    for (int x = 0; x < t; ++x){
        objects.pop_back();
    }
    for (int x = 0; x < temp_objects.size(); ++x){
        objects.push_back(temp_objects[x]);
    }

    for (int x = 0; x < objects.size(); ++x){
        if (orange_loc != -360){
            if ((-1 * (std::get<0>(objects[x]) - 180)) >= orange_loc && (-1 * (std::get<1>(objects[x]) - 180)) <= orange_loc){
                // ROS_INFO_STREAM(" paper loc: " + std::to_string(orange_loc) + 
                //          " Head x: " + std::to_string(-1 * (std::get<0>(objects[x]) - 180)) +
                //          " Tail x: " + std::to_string(-1 * (std::get<1>(objects[x]) - 180)) +
                //          " mid point: " + std::to_string((-1 * (std::get<0>(objects[x]) - 180) + -1 * (std::get<1>(objects[x]) - 180)) / 2) +
                //          " range: " + std::to_string(std::get<0>(std::get<2>(objects[x])))
                //  );
                calculate_move(objects[x]);
                break;
            }
        }
        if(x + 1 == objects.size()){
            stop();
        }
        
    }
    ROS_INFO_STREAM("--------------");
    
}

void CollabVehicleBehavior::imageCallback(const sensor_msgs::ImageConstPtr& image){
    cv_bridge::CvImagePtr rgb_image;
    try{
        rgb_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    
    cv::Mat hsv_image;
    cv::Mat mask;
    cv::cvtColor(rgb_image->image, hsv_image, cv::COLOR_BGR2HSV);
    

    double lo_blue = hue_low_ / 2.0; // OpenCV uses 0-179
    double hi_blue = hue_high_ / 2.0;
    cv::inRange(hsv_image,cv::Scalar(lo_blue, sat_low_, lum_low_),cv::Scalar(hi_blue, sat_high_, lum_high_),mask);
    
    cv::Mat mask_dilated;

    cv::Mat element =\
        cv::getStructuringElement(cv::MORPH_RECT, 
                                  cv::Size(2*dilation_size_+1,2*dilation_size_+1),
                                  cv::Point(dilation_size_,dilation_size_));
    
    dilate(mask,mask_dilated,element);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(mask_dilated, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    int idx = 0;
    int largest_area = largest_area_;
    std::vector<cv::Point> one_contour;
    if(hierarchy.size() > 0){
        for(;idx >= 0; idx = hierarchy[idx][0]){
            if(contourArea(contours[idx]) > largest_area){
                largest_area = contourArea(contours[idx]);
                one_contour = contours[idx];
            }
        }
        if(!one_contour.empty()){
            cv::Rect box = cv::boundingRect(one_contour);
            cv::rectangle(mask_dilated, box, (255,0,255), 2);
            orange_loc = (box.x + (box.width / 2.0000) - 320.0000) * .1875;
        }
        else{
            orange_loc = -360;
        }
    }

    cv::imshow(CVWIN_PREVIEW_1, mask_dilated);
    cv::waitKey(3);
}

void CollabVehicleBehavior::calculate_move(std::tuple<int, int, std::tuple<float, int>> object){
    geometry_msgs::Twist twist;
    float speed_change;

    if(first_loop == 0){
        first_loop++;
        distance = std::get<0>(std::get<2>(object));
        last_time = ros::Time::now();
        return;
    }
    else{
        first_loop++;
        float change_time = (ros::Time::now() - last_time).toSec();
        float change_distance = std::get<0>(std::get<2>(object)) - distance;
        change_over.push_back(change_distance / change_time);
        last_time = ros::Time::now();
        if(first_loop <= 3)
            return;
        change_over.erase(change_over.begin());
        float temp = 0;
        for (std::vector<float>::iterator it = change_over.begin(); it != change_over.end(); ++it){
            temp += *it;
        }
        speed_change = temp/3;
    }

    float mid = (-1 * (std::get<0>(object) - 180) + -1 * (std::get<1>(object) - 180)) / 2;

    twist.linear.x = speed_change +  2 * (std::get<0>(std::get<2>(object)) - .3);
    //ROS_INFO_STREAM(speed_change + 2 * (std::get<0>(std::get<2>(object)) - .3));

    //ROS_INFO_STREAM(-.2 * mid);
    ROS_INFO_STREAM(-1 * (pow(2,.03*orange_loc) - 1));
    

    twist.angular.z = -1 * (pow(2,.03*orange_loc) - 1);

    twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
    pub_.publish(twist);

    /*
    ROS_INFO_STREAM(
        " Distance: " + std::to_string(std::get<0>(std::get<2>(object)) * 100) + " cm" + 
        " | paper: " + std::to_string(orange_loc) + 
        " | mid point: " + std::to_string(mid) +
        " | difrence: " + std::to_string(mid - orange_loc)
    );
    */
}

void CollabVehicleBehavior::stop(){
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.angular.z = 0;
    twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
    pub_.publish(twist);
    ROS_INFO_STREAM("STOP");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "collab vehicle behavior");

    CollabVehicleBehavior sd{};
 
    ROS_INFO_STREAM("CVB running!");
    ros::spin();
    return 0;
}