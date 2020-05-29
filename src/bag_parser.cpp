//
// Created by prashant on 5/27/20.
//

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

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

    rosbag::View* rbg_viewptr = new rosbag::View(bag, rosbag::TopicQuery(std::string("/camera/rgb/image_color")));
    rosbag::View* depth_viewptr = new rosbag::View(bag, rosbag::TopicQuery(std::string("/camera/depth/image")));

    sensor_msgs::Image rgb_image;
    sensor_msgs::Image depth_image;

    for(std::pair<rosbag::View::iterator, rosbag::View::iterator> i(rbg_viewptr->begin(), depth_viewptr->begin());
            i.first != rbg_viewptr->end() && i.second != depth_viewptr->end(); ++i.first, ++i.second){

        rosbag::MessageInstance rgb_instance = *i.first;
        rosbag::MessageInstance depth_instance = *i.second;

        if (depth_instance.getTopic().find("depth") != std::string::npos) {
            sensor_msgs::Image::ConstPtr s = depth_instance.instantiate<sensor_msgs::Image>();
            if(s != NULL) {
                std::cout << "Got KF Depth message \t" << s->header.seq << std::endl;
                depth_pub.publish(s);
            }
        }
        if( rgb_instance.getTopic().find("rgb") != std::string::npos ){
            sensor_msgs::Image::ConstPtr s = rgb_instance.instantiate<sensor_msgs::Image>();
            if(s != NULL) {
                std::cout << "Got KF Color message \t" << s->header.seq << std::endl;
                rgb_pub.publish(s);
            }
        }

        rosbag::View::iterator rgb_ref_img_itr = i.first;
        rosbag::View::iterator depth_ref_img_itr = i.second;

        for(unsigned int i = 0; i<number_of_image_before_kf; ++i){

            rgb_ref_img_itr++;
            depth_ref_img_itr++;

            rosbag::MessageInstance rgb_ref = *rgb_ref_img_itr;
            rosbag::MessageInstance depth_ref = *depth_ref_img_itr;
            if (depth_ref_img_itr->getTopic().find("depth") != std::string::npos) {
                sensor_msgs::Image::ConstPtr s = depth_ref.instantiate<sensor_msgs::Image>();
                if(s != NULL) {
                    std::cout << "Got REF Depth message \t" << s->header.seq << std::endl;
                    depth_pub.publish(s);
                }
            }
            if( rgb_ref_img_itr->getTopic().find("rgb") != std::string::npos ){
                sensor_msgs::Image::ConstPtr s = rgb_ref.instantiate<sensor_msgs::Image>();
                if(s != NULL) {
                    std::cout << "Got REF Color message \t" << s->header.seq << std::endl;
                    rgb_pub.publish(s);
                }
            }
        }
    }

}

