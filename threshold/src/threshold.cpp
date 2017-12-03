#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

//trackbars
void on_trackbar_low_r(int, void *);
void on_trackbar_high_r(int, void *);
void on_trackbar_low_g (int, void *);
void on_trackbar_high_g (int, void *);
void on_trackbar_low_b (int, void *);
void on_trackbar_high_b (int, void *);

cv::Mat frame = Mat::zeros( 200, 200,CV_8UC3);
 Mat thresholded; 

//initial values
int low_r=90, low_g=90, low_b=90;
int high_r=230, high_g=230, high_b=230;
int low_r_tmp=90, low_g_tmp=90, low_b_tmp=90;
int high_r_tmp=230, high_g_tmp=230, high_b_tmp=230;


 int main( int argc, char **argv)
 {
    
    //basic ros stuff
    ros::init(argc, argv, "threshold");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);   
    image_transport::Publisher pub = it.advertise("/lane_image", 1);
    
    //scaning
    image_transport::Subscriber sub = it.subscribe("/top_view", 20, imageCallback);

    //Create trackbars in "Control" window
    namedWindow("Trackbar", 1);
    createTrackbar( "LowR", "Trackbar", &low_r_tmp, 255, on_trackbar_low_r );
    createTrackbar( "HighR", "Trackbar", &high_r_tmp, 255, on_trackbar_high_r );
    createTrackbar( "LowG", "Trackbar", &low_g_tmp, 255, on_trackbar_low_g );
    createTrackbar( "HighG", "Trackbar", &high_g_tmp, 255, on_trackbar_high_g );
    createTrackbar( "LowB", "Trackbar", &low_b_tmp, 255, on_trackbar_low_b );
    createTrackbar( "HighB", "Trackbar", &high_g_tmp, 255, on_trackbar_high_b );
    ros::Rate loop_rate(30);

    while(n.ok())
    {
        on_trackbar_low_r(low_r_tmp, 0);
        on_trackbar_high_r(high_r_tmp, 0);
        on_trackbar_low_g (low_g_tmp, 0);
        on_trackbar_high_g (high_g_tmp, 0);
        on_trackbar_low_b (low_b_tmp, 0);
        on_trackbar_high_b (high_b_tmp, 0);
        
        sensor_msgs::ImagePtr msg;
        msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded).toImageMsg();
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

   return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      frame  = cv_bridge::toCvShare(msg, "bgr8")->image;
      
      if (!frame.empty())
      {
        inRange(frame, Scalar(low_r, low_g, low_b), Scalar(high_r, high_g, high_b), thresholded);
         imshow("thresholded",thresholded);
      }
      
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void on_trackbar_low_r(int, void *)
{
    // low_r = min(high_r-1, low_r);
    low_r = low_r_tmp;
    // setTrackbarPos("Low R","Object Detection", low_r);
}
void on_trackbar_high_r(int , void *)
{
    // high_r = max(high_r, low_r+1);
    high_r = high_r_tmp;
    // setTrackbarPos("High R", "Object Detection", high_r);
}
void on_trackbar_low_g(int, void *)
{
    // low_g = min(high_g-1, low_g);
    low_g = low_g_tmp;
    // setTrackbarPos("Low G","Object Detection", low_g);
}
void on_trackbar_high_g(int , void *)
{
    // high_g = max(high_g, low_g+1);
    high_g = high_g_tmp;
    // setTrackbarPos("High G", "Object Detection", high_g);
}
void on_trackbar_low_b(int, void *)
{
    // low_b= min(high_b-1, low_b);
    low_b = low_b_tmp;
    // setTrackbarPos("Low B","Object Detection", low_b);
}
void on_trackbar_high_b(int, void *)
{
    // high_b = max(high_b, low_b+1);
    high_b = high_b_tmp;
    // setTrackbarPos("High B", "Object Detection", high_b);
}


