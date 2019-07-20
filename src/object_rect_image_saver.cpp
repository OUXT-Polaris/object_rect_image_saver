// Headers in this package
#include <object_rect_image_saver/object_rect_image_saver.h>

ObjectRectImageSaver::ObjectRectImageSaver(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    image_count_ = 0;
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("image_topic", image_topic_, "image_raw");
    pnh_.param<std::string>("rect_topic", rect_topic_, "rect");
    pnh_.param<std::string>("save_prefix", save_prefix_, "save_prefix");
    pnh_.param<int>("min_height", min_height_, 0);
    pnh_.param<int>("min_width", min_width_, 0);
    pnh_.param<int>("margin", margin_, 10);
    namespace fs = boost::filesystem;
    const fs::path path(save_prefix_);
    boost::system::error_code error;
    const bool result = fs::create_directories(path, error);
    if (!result || error)
    {
        ROS_WARN_STREAM("failed to make directory.");
    }
    image_sub_ptr_ = std::unique_ptr<ImageSubscriber>(new ImageSubscriber(nh_, image_topic_, 10));
    rect_sub_ptr_ = std::unique_ptr<RectSubscriber>(new RectSubscriber(nh_, rect_topic_, 10));
    sync_ptr_ = std::unique_ptr<Synchronizer>(new Synchronizer(SyncPolicy(10),*image_sub_ptr_,*rect_sub_ptr_));
    sync_ptr_->registerCallback(boost::bind(&ObjectRectImageSaver::callback, this, _1, _2));
}

ObjectRectImageSaver::~ObjectRectImageSaver()
{

}

void ObjectRectImageSaver::callback(const sensor_msgs::Image::ConstPtr image,const jsk_recognition_msgs::RectArray::ConstPtr rect)
{
    ROS_INFO_STREAM("image and rect recieved");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat cv_image = cv_ptr->image;
    int height = cv_image.rows;
    int width = cv_image.cols;
    for(auto itr = rect->rects.begin(); itr != rect->rects.end(); itr++)
    {
        if(itr->height > min_height_ && itr->width > min_width_)
        {
            if((itr->x+itr->width)<0)
            {
                continue;
            }
            if((itr->y+itr->height)<0)
            {
                continue;
            }
            try
            {
                cv::Rect cv_rect{itr->x-margin_, itr->y-margin_, itr->width+2*margin_, itr->height+2*margin_};
                if((cv_rect.x+cv_rect.width) > width)
                {
                    cv_rect.width = width-cv_rect.x;
                }
                if((cv_rect.y+cv_rect.height) > height)
                {
                    cv_rect.height = height-cv_rect.y;
                }
                if(cv_rect.x < 0)
                {
                    cv_rect.x = 0;
                }
                if(cv_rect.y < 0)
                {
                    cv_rect.y = 0;
                }
                cv::Mat cv_cut_img(cv_image,cv_rect);
                std::string image_filename = save_prefix_+"/"+std::to_string(image_count_)+".png";
                bool result = cv::imwrite(image_filename,cv_cut_img);
                if(result)
                {
                    ROS_INFO_STREAM(image_filename << " saved");
                    image_count_ = image_count_ + 1;
                }
                else
                {
                    ROS_WARN_STREAM("failed to write " << image_filename);
                }
            }
            catch(...)
            {

            }
        }
    }
}