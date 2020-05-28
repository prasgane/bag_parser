//
// Created by prashant on 5/27/20.
//
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

int main(int argc, char **argv){

    ros::init(argc, argv, "bag_parser");
    rosbag::Bag bag;
    std::string bag_path("/home/prashant/Downloads/rgbd_dataset_freiburg2_pioneer_360.bag");
    bag.open(bag_path, rosbag::bagmode::Read);

    ros::NodeHandle nh_("");

    int number_of_image_before_kf;
    nh_.param<int>("num_image_btw_kf", number_of_image_before_kf,30);
    ros::Publisher rgb_pub = nh_.advertise<sensor_msgs::Image>("camera/rgb/image_raw",10);
    ros::Publisher depth_pub = nh_.advertise<sensor_msgs::Image>("camera/depth/image_raw",10);

    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/image_color"));
    topics.push_back(std::string("/camera/depth/image"));

    rosbag::View* viewptr = new rosbag::View(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator start_iter, end_itr, reference_image_itr;
    start_iter = viewptr->begin();
    end_itr = viewptr->end();
    reference_image_itr = start_iter;

    sensor_msgs::Image rgb_image;
    sensor_msgs::Image depth_image;

    // Will iterate through the bag
    for(rosbag::View::iterator kf_itr = start_iter; kf_itr != end_itr; ++kf_itr){

        rosbag::MessageInstance m = *kf_itr;
        if (m.getTopic().find("depth") != std::string::npos) {
            sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
            if(s != NULL) {
                std::cout << "Got KF Depth message \t" << s->header.seq << std::endl;
                depth_pub.publish(s);
            }
        }
        if( m.getTopic().find("rgb") != std::string::npos ){
            sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
            if(s != NULL) {
                std::cout << "Got KF Color message \t" << s->header.seq << std::endl;
                rgb_pub.publish(s);
            }
        }

        rosbag::View::iterator ref_img_itr = kf_itr;
        for(unsigned int i = 0; i<number_of_image_before_kf; ++i){
            ref_img_itr++;
            rosbag::MessageInstance m = *ref_img_itr;
            sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
            if (m.getTopic().find("depth") != std::string::npos) {
                if(s != NULL) {
                    std::cout << "Got Reference Depth message \t" << s->header.seq << std::endl;
//                    depth.publish(s);
                }
            }
            if( m.getTopic().find("rgb") != std::string::npos ){
                if(s != NULL) {
                    std::cout << "Got Reference Color message \t" << s->header.seq << std::endl;
//                    rgb_pub.publish(s);
                }
            }
        }


    }

}

