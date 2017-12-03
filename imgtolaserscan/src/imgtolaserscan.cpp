#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include <iostream> 

using namespace std;
#define PI 3.14159265

int z = 0;
int width=1280,height=498;
int r_max = sqrt((width/2)*(width/2) + (height-z)*(height-z));
int theta0 = atan(2*z/width) * 180 / PI ;
double increment = PI/180;

const int num_readings = 181;
double ranges[num_readings];
double ans[181]={r_max-1};

int count = 0;

void msgRecieved(const sensor_msgs::ImageConstPtr& msg)
{
	try 
    {
    	for(int i=0;i<181;i++)
		{
			if(i%2)
			 ans[i]=0;
			else ans[i]=30000;
		}
    	cv::Mat gs(cv_bridge::toCvShare(msg, "bgr8")->image.rows,cv_bridge::toCvShare(msg, "bgr8")->image.cols,CV_8UC1);
		cv::cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image, gs, CV_BGR2GRAY);

		if(!gs.empty())
		{
			cv::threshold(gs,gs,100,255,0);
			cv::imshow("GrayScale", gs);
		}	
		// top - perspective

				
	
		for (int i = 0; i < gs.cols; ++i)
		{
			for (int j = 0; j < gs.rows; ++j)
			{
				if ((int)gs.at<uchar>(j,i))
				{	int x=i-gs.cols/2;
					int y=gs.rows+z-j;
					int theta =floor(atan2(y,x)*180/PI);
					
					if (theta%2)
					{
						if ((x*x+y*y)>ans[theta]*ans[theta])
						{
							ans[theta]=sqrt(x*x+y*y);
							
						}
					}
					 else{
					 	if ((x*x+y*y)<ans[theta]*ans[theta])
					 	{
					 		ans[theta]=sqrt(x*x+y*y);
					
					 	}
					 }
				}
				//cout<< (int)gs.at<uchar>(10,110) << " "<<(int)gs.at<uchar>(10,90);
			}
			
		}
		for (int i = 0; i < 181; ++i)
		{
			ranges[i]=0.05 * ans[i];
		}
    	
    	cv::waitKey(30);
  	}
  	catch (cv_bridge::Exception& e)
  	{
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"imgtolaserscan");
	ros::NodeHandle n;
	sensor_msgs::LaserScan scan;
	cv::namedWindow("GrayScale",1); 
	cv::startWindowThread();
	image_transport::ImageTransport	it(n);
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan_lane", 50);
	image_transport::Subscriber sub = it.subscribe("/lane_image",1,msgRecieved);
	ros::Rate loop_rate(30);
	while(n.ok())
	{
		ros::Time scan_time = ros::Time::now();
	    //populate the LaserScan message
	    scan.header.stamp = scan_time;
	    scan.header.frame_id = "laser_frame";
	    scan.angle_min = 0;
	    scan.angle_max = PI;
	    scan.angle_increment = increment;
	    scan.time_increment = 0;
	    scan.range_min = 1.0;
	    scan.range_max = r_max;

	    scan.ranges.resize(num_readings);
	    scan.intensities.resize(num_readings);
	    for(unsigned int i = 0; i < num_readings; ++i){
	      scan.ranges[i] = ranges[i];
	    }

	    scan_pub.publish(scan);

		ros::spinOnce();
		loop_rate.sleep();
	}
	cv::destroyWindow("GrayScale");
	return 0;
}


