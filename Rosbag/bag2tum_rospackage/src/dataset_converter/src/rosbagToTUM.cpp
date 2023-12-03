//
// Created by ywl on 22-9-18.
//
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h>
#include <fstream>
#include <exception>
#include <std_srvs/Trigger.h>

void showProgress(float progress);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbagToTUM");
    ros::NodeHandle n;



    std::string bag_name;
    std::string topic_name_rgb, topic_name_depth;
    std::string outputPath;

    n.setParam("convertFinish", 0);
    n.param<std::string>("topic_name_rgb", topic_name_rgb, "/camera/color/image_raw");
    n.param<std::string>("topic_name_depth", topic_name_depth, "/camera/aligned_depth_to_color/image_raw");
    n.param<std::string>("bag_name", bag_name, "/home/ywl/dataset/indoor.bag");
    n.param<std::string>("outputpath", outputPath, "/home/ywl/rosbagToTUM");



    mkdir(outputPath.c_str(), S_IRWXU );
    mkdir((outputPath+"/dataset").c_str(),  S_IRWXU);
    mkdir((outputPath+"/dataset/rgb").c_str(), S_IRWXU );
    mkdir((outputPath+"/dataset/depth").c_str(), S_IRWXU);

    if (access(outputPath.c_str(), F_OK) == -1 ||
            access((outputPath+"/dataset").c_str(), F_OK) == -1 ||
            access((outputPath+"/dataset/rgb").c_str(), F_OK )== -1 ||
            access((outputPath+"/dataset/depth").c_str(), F_OK) == -1)
    {
        std::cerr << "Failed to create output DIR!" << std::endl;
        return -1;
    }

    rosbag::Bag bag;
    bag.open(bag_name);
    ROS_INFO("Successfully open %s", bag_name.c_str());

    std::vector<std::string> topics;
    topics.push_back(topic_name_rgb);

    ROS_INFO("Processing RGB image ...");
    std::cout << std::endl;

    std::fstream f_rgb;
    f_rgb.open(outputPath+"/dataset/rgb.txt", std::ios::out);
    f_rgb << "# color images" << std::endl;
    f_rgb << "# file: " << bag_name << std::endl;
    f_rgb << "# timestamp filename" << std::endl;

    rosbag::View viewRGB(bag, rosbag::TopicQuery(topics));
    float count_rgb = 0.0;
    for (rosbag::MessageInstance const msg : viewRGB)
    {
        sensor_msgs::ImageConstPtr pRGB = msg.instantiate<sensor_msgs::Image>();
        if (pRGB != nullptr)
        {
            std::string timeStamp = std::to_string(pRGB->header.stamp.toSec());
            cv::Mat mRGB = cv_bridge::toCvCopy(pRGB)->image;
            cv::imwrite(outputPath + "/dataset/rgb/" + timeStamp + ".png", mRGB);
            // ROS_INFO("Have written RGB image %s/dataset/rgb/%s.png", outputPath.c_str(), timeStamp.c_str());
            f_rgb << timeStamp << " " << "rgb/" << timeStamp+".png" << std::endl;
        }

        showProgress(count_rgb/viewRGB.size());
        count_rgb += 1.0;

    }
    f_rgb.close();
    topics.clear();
    topics.push_back(topic_name_depth);



    ROS_INFO("Processing Depth image ...");
    std::cout << std::endl;

    std::fstream f_depth;
    f_depth.open(outputPath+"/dataset/depth.txt", std::ios::out);
    f_depth << "# depth maps" << std::endl;
    f_depth << "# file: " << bag_name << std::endl;
    f_depth << "# timestamp filename" << std::endl;

    rosbag::View viewDepth(bag, rosbag::TopicQuery(topics));
    float count_depth = 0.0;

    for (rosbag::MessageInstance const msg : viewDepth)
    {
        sensor_msgs::ImageConstPtr pDepth = msg.instantiate<sensor_msgs::Image>();
        if (pDepth != nullptr)
        {
            std::string timeStamp = std::to_string(pDepth->header.stamp.toSec());
            cv::Mat mDepth = cv_bridge::toCvCopy(pDepth)->image;
            cv::imwrite(outputPath + "/dataset/depth/" + timeStamp + ".png", mDepth);
            // ROS_INFO("Have written Depth image %s/dataset/depth/%s.png", outputPath.c_str(), timeStamp.c_str());
            f_depth << timeStamp << " " << "depth/" << timeStamp+".png" << std::endl;
        }

        showProgress(count_depth/viewDepth.size());
        count_depth += 1.0;

    }
    std::cout << std::endl;
    f_depth.close();

    //ROS_INFO("Successfully convert %s to TUM dataset \n Totally RGB image x %i, Depth image x %i", bag_name.c_str(), int(count_rgb), int(count_depth));
    std::cout << "Successfully convert" << bag_name << "to TUM dataset" << std::endl;
    std::cout << "################################" << std::endl;
    std::cout << "RGB image" << "                  " << count_rgb << std::endl;
    std::cout << "Depth image" << "                " << count_depth << std::endl;
    std::cout << "################################" << std::endl;

    bag.close();


    ros::service::waitForService("/associate");
    ros::ServiceClient associate = n.serviceClient<std_srvs::Trigger>("/associate");
    std_srvs::Trigger srv;
    associate.call(srv);


    return 0;
}

void showProgress(float progress)
{
    if (progress > 1)
        progress = 1;
    int pa = progress * 50;
    std::cout << "\33[1A";
    std::cout << "[" + std::string(pa, '=') + ">" + std::string(50 - pa, ' ') << "]  " << progress * 100 << "% " << std::endl;
    fflush(stdout);
}
