#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include<iostream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat src = cv_ptr->image;
        cv::imshow("source", src);
        cv::waitKey(30);

        int w = src.size().width;
        int h = src.size().height;
        std::cout<<"Width: "<<w<<"\n";
        std::cout<<"Height: "<<h<<"\n";
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("source");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/webcam/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("source");
}