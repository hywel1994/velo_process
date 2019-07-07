#include <ctime>
#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iamgetest");
    ros::NodeHandle m_nh;

    image_transport::Publisher m_image_label_pub;

    image_transport::ImageTransport imageTrans(m_nh);
    m_image_label_pub = imageTrans.advertise("test", 1);
    cv::Mat image = cv::imread("/home/hywel/lidar_annotation/image/000000.png");
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate loop_rate(5);
    while (m_nh.ok())
    {
        m_image_label_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}