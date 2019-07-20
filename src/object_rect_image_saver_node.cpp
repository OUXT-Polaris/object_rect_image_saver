// Headers in this package
#include <object_rect_image_saver/object_rect_image_saver.h>

// Headers in ROS
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nmea_to_geopose_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ObjectRectImageSaver saver(nh,pnh);
    ros::spin();
    return 0;
}