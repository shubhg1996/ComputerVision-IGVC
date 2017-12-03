#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <std_msgs/Int16.h>

using namespace std;
using namespace cv;

cv::VideoCapture cap;

void msgCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat image;
    image = cv_bridge::toCvShare(msg)->image;
    if(!image.empty())
    {
    //cv::cvtColor(image, image, CV_BGR2GRAY);
    cv::imshow("swati", image);
    //cv::waitkey(0);
    return;
  }
  return;
  }
  int main(int argc, char** argv)
  {
      ros::init(argc, argv, "gray_driver");
      ros::NodeHandle nh;

      image_transport::ImageTransport it(nh);
      image_transport::Subscriber sub = it.subscribe("lane_image", 1000, msgCallback);


      //cap.set(3, 640);
      //cap.set(4, 480);

       cv::namedWindow("swati");
       cv::startWindowThread();
       ros::Rate loop_rate(30);

       ros::spin();


   return 0;
  }
