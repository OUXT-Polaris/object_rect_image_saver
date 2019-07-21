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
        ROS_WARN_STREAM("failed to make directory. :");
    }
    const fs::path path_raw(save_prefix_+"/.raw");
    const bool result_raw = fs::create_directories(path_raw, error);
    if (!result_raw || error)
    {
        ROS_WARN_STREAM("failed to make directory. :" << save_prefix_ << "/.raw");
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
    bool write_succeed = false;
    int count = 0;
    std::vector<std::pair<cv::Rect,std::string> > rect_image_pairs;
    for(auto itr = rect->rects.begin(); itr != rect->rects.end(); itr++)
    {
        if(itr->height > min_height_ && itr->width > min_width_)
        {
            if((itr->x+itr->width)<0 || (itr->x+itr->width)>width)
            {
                continue;
            }
            if((itr->y+itr->height)<0 || (itr->y+itr->height)>height)
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
                std::string image_filename = save_prefix_+"/"+std::to_string(image_count_)+"_"+std::to_string(count)+".png";
                bool result = cv::imwrite(image_filename,cv_cut_img);
                if(result)
                {
                    ROS_INFO_STREAM(image_filename << " saved");
                    std::pair<cv::Rect,std::string> rect_image_pair = std::make_pair(cv_rect,image_filename);
                    rect_image_pairs.push_back(rect_image_pair);
                    count = count + 1;
                    write_succeed = true;
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
    if(write_succeed)
    {
        std::string image_filename = save_prefix_+"/.raw/"+std::to_string(image_count_)+".png";
        bool result = cv::imwrite(image_filename,cv_image);
        if(!result)
        {
            ROS_WARN_STREAM("failed to save raw image : " << image_filename);
            return;
        }
        else
        {
            ROS_INFO_STREAM("save to " << image_filename);
        }
        image_count_ = image_count_ + 1;
        boost::property_tree::ptree image_tree;
        int cnt = 0;
        for(auto itr = rect_image_pairs.begin(); itr != rect_image_pairs.end(); itr++)
        {
            boost::property_tree::ptree rect_tree;
            std::string key = std::to_string(cnt) + ".rect.x";
            rect_tree.put(key, itr->first.x);
            key = std::to_string(cnt) + ".rect.y";
            rect_tree.put(key, itr->first.y);
            key = std::to_string(cnt) + ".rect.width";
            rect_tree.put(key, itr->first.width);
            key = std::to_string(cnt) + ".rect.height";
            rect_tree.put(key, itr->first.height);
            image_tree.push_back(std::make_pair("",rect_tree));
            cnt = cnt + 1;
        }
        pt_.add_child(std::to_string(image_count_), image_tree);
    }
}

void ObjectRectImageSaver::outputAnnotation()
{
    ROS_INFO_STREAM("output annotation file");
    write_json(save_prefix_+"/annotation.json", pt_);
    return;
}