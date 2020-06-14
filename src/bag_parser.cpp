//
// Created by prashant on 5/27/20.
//

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

void writebag(sensor_msgs::Image rgb, sensor_msgs::Image depth,
        sensor_msgs::CameraInfo rgb_info, sensor_msgs::CameraInfo depth_info,
        std::string outputfile){

    depth_info.header = depth.header;
    rgb_info.header = rgb.header;

    rosbag::Bag output_bag;
    output_bag.open(outputfile, rosbag::bagmode::Append);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "bag_parser");
    rosbag::Bag bag;
    ros::NodeHandle nh_("");

    int number_of_image_before_kf;
    std::string bag_path;
    nh_.param<int>("num_image_btw_kf", number_of_image_before_kf, 30);
//    nh_.param<std::string>("bag_path", bag_path, "/home/prashant/Downloads/rgbd_dataset_freiburg2_pioneer_360.bag");
    nh_.param<std::string>("bag_path", bag_path, "/home/prashant/Downloads/20200124_006.bag");
    ROS_WARN_STREAM("BAG_PARSER::Image between KF is \t" << number_of_image_before_kf);

    ros::Publisher rgb_pub = nh_.advertise<sensor_msgs::Image>("camera/rgb/image_color", 10);
    ros::Publisher depth_pub = nh_.advertise<sensor_msgs::Image>("camera/depth/image", 10); //_registered
    ros::Publisher rgb_camera_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("camera/rgb/camera_info", 10);
    ros::Publisher depth_camera_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("camera/depth/camera_info", 10);

    bag.open(bag_path, rosbag::bagmode::Read);
//    rosbag::View *rbg_viewptr = new rosbag::View(bag, rosbag::TopicQuery(std::string("/camera/rgb/image_color")));
//    rosbag::View *depth_viewptr = new rosbag::View(bag, rosbag::TopicQuery(std::string("/camera/depth/image")));
//    rosbag::View *rgb_cam_info = new rosbag::View(bag, rosbag::TopicQuery(std::string("/camera/rgb/camera_info")));
//    rosbag::View *depth_cam_info = new rosbag::View(bag, rosbag::TopicQuery(std::string("/camera/depth/camera_info")));
    rosbag::View *rbg_viewptr = new rosbag::View(bag, rosbag::TopicQuery(std::string("camera/rgb/image_raw")));
    rosbag::View *depth_viewptr = new rosbag::View(bag, rosbag::TopicQuery(std::string("camera/depth_registered/image_raw")));
    rosbag::View *rgb_cam_info = new rosbag::View(bag, rosbag::TopicQuery(std::string("camera/rgb/camera_info")));
    rosbag::View *depth_cam_info = new rosbag::View(bag, rosbag::TopicQuery(std::string("camera/depth_registered/camera_info")));
    sensor_msgs::Image rgb_image;
    sensor_msgs::Image depth_image;

    rosbag::View::iterator rgb_itr, depth_itr;
    rgb_itr = rgb_cam_info->begin();
    depth_itr = depth_cam_info->begin();

    while (ros::ok()) {
        for (std::pair<rosbag::View::iterator, rosbag::View::iterator> i(rbg_viewptr->begin(), depth_viewptr->begin());
             i.first != rbg_viewptr->end() && i.second != depth_viewptr->end(); ++i.first, ++i.second) {
            rosbag::MessageInstance rgb_instance = *i.first;
            rosbag::MessageInstance depth_instance = *i.second;

            std_msgs::Header rgb_header;
            sensor_msgs::CameraInfo rgb_info, depth_info;
            sensor_msgs::Image depth_temp;

            if (rgb_instance.getTopic().find("rgb") != std::string::npos) {
                sensor_msgs::Image::ConstPtr image_rgb = rgb_instance.instantiate<sensor_msgs::Image>();
                if (image_rgb != NULL) {
                    rgb_pub.publish(image_rgb);
                    rgb_header = image_rgb->header;
                }
            }

            if (depth_instance.getTopic().find("depth") != std::string::npos) {
                sensor_msgs::Image::ConstPtr image_depth = depth_instance.instantiate<sensor_msgs::Image>();
                if (image_depth != NULL) {
                    depth_temp = *image_depth;
                    depth_temp.header = rgb_header;
                    depth_pub.publish(image_depth);
                }
            }
            rosbag::MessageInstance rgb_cam_info_inst = *rgb_itr;
            rosbag::MessageInstance depth_cam_info_inst = *depth_itr;

            if (rgb_cam_info_inst.getTopic().find("rgb") != std::string::npos) {
                sensor_msgs::CameraInfo::ConstPtr info = rgb_cam_info_inst.instantiate<sensor_msgs::CameraInfo>();
                if (info != NULL) {
                    rgb_info = *info;
                    rgb_info.header = rgb_header;
                    rgb_camera_info_pub.publish(rgb_info);
                }
            }

            if (depth_cam_info_inst.getTopic().find("depth") != std::string::npos) {
                sensor_msgs::CameraInfo::ConstPtr info = depth_cam_info_inst.instantiate<sensor_msgs::CameraInfo>();
                if (info != NULL) {
                    depth_info = *info;
                    depth_info.header = rgb_header;
                    depth_camera_info_pub.publish(depth_info);
                }
            }

            rosbag::View::iterator rgb_ref_img_itr = i.first;
            rosbag::View::iterator depth_ref_img_itr = i.second;

//            for (unsigned int i = 0; i < number_of_image_before_kf; ++i) {
//
//                rgb_ref_img_itr++;
//                depth_ref_img_itr++;
//
//                rosbag::MessageInstance rgb_ref = *rgb_ref_img_itr;
//                rosbag::MessageInstance depth_ref = *depth_ref_img_itr;
//                if (depth_ref_img_itr->getTopic().find("depth") != std::string::npos) {
//                    sensor_msgs::Image::ConstPtr s = depth_ref.instantiate<sensor_msgs::Image>();
//                    if (s != NULL)
//                        depth_pub.publish(s);
//                }
//                if (rgb_ref_img_itr->getTopic().find("rgb") != std::string::npos) {
//                    sensor_msgs::Image::ConstPtr s = rgb_ref.instantiate<sensor_msgs::Image>();
//                    if (s != NULL) {
//                        rgb_header = s->header;
//                        rgb_pub.publish(s);
//                    }
//                }
//                depth_info.header = rgb_header;
//                depth_camera_info_pub.publish(depth_info);
//                rgb_info.header = rgb_header;
//                rgb_camera_info_pub.publish(rgb_info);
//
//            }
        }
        bag.close();
    }
}

