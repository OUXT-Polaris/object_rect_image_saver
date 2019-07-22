#ifndef OBJECT_RECT_IMAGE_SAVER_H_OBJECT_RECT_IMAGE_SAVER_H_INCLUDED
#define OBJECT_RECT_IMAGE_SAVER_H_OBJECT_RECT_IMAGE_SAVER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

// Headers in STL
#include <memory>

// Headers in OpenCV
#include <opencv2/opencv.hpp>

// Headers in Boost
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
typedef message_filters::Subscriber<jsk_recognition_msgs::RectArray> RectSubscriber;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, jsk_recognition_msgs::RectArray> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

class ObjectRectImageSaver
{
public:
    ObjectRectImageSaver(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~ObjectRectImageSaver();
    void outputAnnotation();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void callback(const sensor_msgs::Image::ConstPtr image,const jsk_recognition_msgs::RectArray::ConstPtr rect);
    std::unique_ptr<ImageSubscriber> image_sub_ptr_;
    std::unique_ptr<RectSubscriber> rect_sub_ptr_;
    std::unique_ptr<Synchronizer> sync_ptr_;
    std::string image_topic_;
    std::string rect_topic_;
    std::string save_prefix_;
    int image_count_;
    int min_height_;
    int min_width_;
    int margin_;
    boost::property_tree::ptree pt_;
};

#endif  //OBJECT_RECT_IMAGE_SAVER_H_OBJECT_RECT_IMAGE_SAVER_H_INCLUDED