#include <math.h>
#include <stdlib.h>
#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/LaserScan.h>

using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void laserdata_recieved(const sensor_msgs::LaserScan &LaserScan); //change calibration and transformation here
void obstacleEliminate(Mat temp); //this blacks out stuff
void blackOut(Mat image, float x, float y); //change colour of objects here
void blackOut_big(Mat image, float y, float x);

double c_pos[7] = {0, 0, 0, 0 , 0, 0, 0}; //Initialization

//calibration
int x_tmp = 95, y_tmp = 93, z_tmp= 115, qx_tmp = 100, qy_tmp = 100, qz_tmp = 100, qw_tmp = 100, roll_tmp = 100, pitch_tmp = 100, yaw_tmp = 100;
double roll, pitch, yaw;

void toEulerianAngle(double q_x, double q_y, double q_z, double q_w, double& roll, double& pitch, double& yaw)
{
	double ysqr = q_y * q_y;

	// roll (x-axis rotation)
	double t0 = +2.0 * (q_w * q_x + q_y * q_z);
	double t1 = +1.0 - 2.0 * (q_x * q_x + ysqr);
	roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q_w * q_y - q_z * q_x );
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	pitch = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q_w * q_z + q_x * q_y );
	double t4 = +1.0 - 2.0 * (ysqr + q_z * q_z);  
	yaw = std::atan2(t3, t4);
}

void toQuaternion(double& q_x, double& q_y, double& q_z, double& q_w, double pitch, double roll, double yaw)
{
	double t0 = std::cos(yaw * 0.5);
	double t1 = std::sin(yaw * 0.5);
	double t2 = std::cos(roll * 0.5);
	double t3 = std::sin(roll * 0.5);
	double t4 = std::cos(pitch * 0.5);
	double t5 = std::sin(pitch * 0.5);

	q_w = t0 * t2 * t4 + t1 * t3 * t5;
	q_x = t0 * t3 * t4 - t1 * t2 * t5;
	q_y = t0 * t2 * t5 + t1 * t3 * t4;
	q_z = t1 * t2 * t4 - t0 * t3 * t5;
}

cv::Mat frame = Mat::zeros( 200, 200,CV_8UC3);
// Mat black1 = Mat::zeros( 200, 200,CV_8UC3);
Mat frame2;

float angle_min;
float angle_max;
float angle_increment;

float ranges[360];  // change this with sensor //calculated as (angle_max - angle_min) / angle_increment // 
       			   //stores the angle in order from angle_min -> angle_max
			   //the value represents the distance of objects

float lidar[360][2]; //0th element is z and other is x 
float camera[360][4];
float image_x[360];
float image_y[360];


//following represents intrinsic matrix of the camera
//Logitech
// float k[3][3] = {{ 476.7030836014194, 0.0, 400.5},
// 	            { 0.0, 476.7030836014194, 400.5},
// 	            { 0.0, 0.0, 1.0}};
//Genius
float k[3][3] = {{ 307.9 , 0.0, 343.2},
	            { 0.0, 306.9, 211.0},
	            { 0.0, 0.0, 1.0}};

double tmp3[4][4] = {    {    -0.9976,       0.0543,    -0.0413,      0},
							 {     -0.0504 ,    -0.9468,   -0.8969,      0},
							 {     -0.0459 ,    -0.0874,    0.9951,      0},
							 {           0,          0,   0,      1}};

Mat R = Mat(4, 4, CV_64FC1, &tmp3);	
double tmp4[4][4] = {    {         0,          0 ,                     0,      -0.0087 },
	{                    0,                     0,                      0,     0.00069 },
	{                    0,                     0,                      0,     -0.1180 },
	{                    0,                     0,                      0,                 0} };

Mat trans = Mat(4, 4, CV_64FC1, &tmp4);
Mat tsf = trans;

double test[4][4];

int main( int argc, char **argv )
{
	//cant perform operations outside of main
	tsf =  tsf + R;

	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			test[i][j] = tsf.at<double>(i , j);

	//basic ros stuff
	ros::init(argc, argv, "watershed");
	ros::NodeHandle n;
	
	image_transport::ImageTransport it(n);   
	image_transport::Publisher pub = it.advertise("/obstacle_free_image", 1);
	
	//scaning
	image_transport::Subscriber sub = it.subscribe("/image", 20, imageCallback);
	ros::Subscriber sub_laser = n.subscribe("scan", 3, laserdata_recieved);

	while(n.ok())
	{
		//publishing
		// imshow("With objects1",black1);
		waitKey(5);
		sensor_msgs::ImagePtr msg;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame2).toImageMsg();
		pub.publish(msg);
		
		ros::spinOnce();
	}

	return 0;
}

Mat obstacleEliminate(Mat image,float u,float l,int w,int flag)
{	
	cv::Mat black =  image;

	int positive[360];

	for(int i = 0; i < 360; i++) 
		positive[i] = 0;

	Mat pose, c_tmp;
	for(int i = 0; i != 359 ; i = (i + 1) % 360) //only project what is in front of the lidar
	{
		int iy = floor( image_y[i] );
		int ix = floor( image_x[i] );
		if(ranges[i] < 20 && ix >= 2 && ix< black.cols - 2 && iy >= 2 && iy < black.rows - 2)
		{
			positive[i]=1;
		}
	}

	for(int i = 0; i != 359 ; i = (i + 1) % 360) //only project what is in front of the lidar
	{
		int iy = floor( image_y[i] );
		int ix = floor( image_x[i] );
		
		if(positive[i] == 1 || ( !flag && ((positive[i-1] && positive[i-2] ) || (positive[i+1] && positive[i+2] ))))
		{	
			
			
			double tmp[4][1]= {     { lidar[i][1] },
							{         u     }, //Also for Anay. Top Heihgt.
							{ lidar[i][0] },
							{         1     }  };


				pose = Mat(4, 1, CV_64FC1, &tmp);

				c_tmp = tsf * pose;

				double s = c_tmp.at<double>(3,0);
				double x = c_tmp.at<double>(0,0)  / s;
				double y = c_tmp.at<double>(1,0)  / s;
				double z = c_tmp.at<double>(2,0)  / s;



				int image_y_temp  = ( k[1][0] * x + k[1][1] * y + k[1][2] * z )/ z;  //scaling factor lidar[i][0]  is constant
				
				double tmp2[4][1]= {     { lidar[i][1] },
							{        l  }, //For Anay. Bottom Height
							{ lidar[i][0] },
							{         1     }  };


				pose = Mat(4, 1, CV_64FC1, &tmp2);

				c_tmp = tsf * pose;

				 s = c_tmp.at<double>(3,0);
				 x = c_tmp.at<double>(0,0)  / s;
				 y = c_tmp.at<double>(1,0)  / s;
				 z = c_tmp.at<double>(2,0)  / s;

				int image_y_temp2  = ( k[1][0] * x + k[1][1] * y + k[1][2] * z )/ z;  //scaling factor lidar[i][0]  is constant

			if(((positive[i-1] || positive[i-2] ) && (positive[i+1] || positive[i+2] )) || !flag)
			for(int j = image_y_temp; j < image_y_temp2; j ++)
			{
				for(int k = ix - w; k < ix + w; k++) //width of the angle
					if( k >= 2 && k < black.cols - 2 && j >= 2 && j < black.rows - 2){
					{	
						blackOut(black, j, k);
					}
				}
			}
		}

	}
	
//filling gaps in obstacles
	for(int i = 0; i != 359; i = (i + 1) % 360) //only project what is in front of the lidar
	{
		if( positive[i] == 0 && positive[ i - 1]  == 1 && positive[ i + 1 ] == 1)
		{	
			i = i - 1; //project the previous point

			int iy = floor( image_y[i] );
			int ix = floor( image_x[i] );

			if(ix >= 2 && ix< black.cols - 2 && iy >= 2 && iy < black.rows - 2)
			{

				double tmp[4][1]= {     { lidar[i][1] },
							{         u     },
							{ lidar[i][0] },
							{         1     }  };


				pose = Mat(4, 1, CV_64FC1, &tmp);

				c_tmp = tsf * pose;

				double s = c_tmp.at<double>(3,0);
				double x = c_tmp.at<double>(0,0)  / s;
				double y = c_tmp.at<double>(1,0)  / s;
				double z = c_tmp.at<double>(2,0)  / s;

				int image_y_temp  = ( k[1][0] * x + k[1][1] * y + k[1][2] * z )/ z;  //scaling factor lidar[i][0]  is constant
				
				double tmp2[4][1]= {     { lidar[i][1] },
							{        l    },
							{ lidar[i][0] },
							{         1     }  };


				pose = Mat(4, 1, CV_64FC1, &tmp2);

				c_tmp = tsf * pose;

				 s = c_tmp.at<double>(3,0);
				 x = c_tmp.at<double>(0,0)  / s;
				 y = c_tmp.at<double>(1,0)  / s;
				 z = c_tmp.at<double>(2,0)  / s;

				int image_y_temp2  = ( k[1][0] * x + k[1][1] * y + k[1][2] * z )/ z;  //scaling factor lidar[i][0]  is constant


				for(int j = image_y_temp; j < image_y_temp2; j ++)
				{
					for(int k = ix  - w; k < ix - w; k++) //width of the angle
						if( k >= 2 && k < black.cols - 2 && j >= 2 && j < black.rows - 2)
							blackOut(black, j, k);
				}
			}

			i = i + 1; //switching back original angle
		}
	}

	// black.copyTo(black1);
	
	return black;
}

void blackOut(Mat black, float iy, float ix) //prints 3x3 boxes
{
	blackOut_big(black, iy, ix);
	blackOut_big(black, iy, ix-1);
	blackOut_big(black, iy, ix+1);
	blackOut_big(black, iy - 1, ix);		
	blackOut_big(black, iy + 1, ix);
	blackOut_big(black, iy - 1, ix-1);
	blackOut_big(black, iy + 1, ix+1);
	blackOut_big(black, iy - 1, ix+1);
	blackOut_big(black, iy + 1, ix-1);
}

void blackOut_big(Mat image, float y, float x) //prints individual pixels
{
	image.at<Vec3b>( y, x)[0]  = 0; //color
	image.at<Vec3b>( y, x)[1]  = 0;
	image.at<Vec3b>( y, x)[2]  = 0;
}

void laserdata_recieved(const sensor_msgs::LaserScan &LaserScan) //and processed
{
	angle_min = LaserScan.angle_min;
	angle_max = LaserScan.angle_max;
	angle_increment = LaserScan.angle_increment;

	Mat pose; 
	Mat c_tmp;
	double x, y, z, s;

	for(int i = 0; i < 360; i++)
	{
		ranges[i] = LaserScan.ranges[i];

		//to cartesian coordinates
		//z
		lidar[i][0] = ranges[i] * cos(  (i) * angle_increment + angle_min);
		//x
		lidar[i][1] = ranges[i] * sin( (i) * angle_increment + angle_min ); //x and y coordinates in m


	
		double tmp[4][1]= {     { lidar[i][1] }, //x
					{         0     },			//y
					{ lidar[i][0] },	//z
					{         1     }  }; //scaling


		pose = Mat(4, 1, CV_64FC1, &tmp);

		c_tmp = tsf * pose;

		s = c_tmp.at<double>(3,0);
		x = c_tmp.at<double>(0,0)  / s;
		y = c_tmp.at<double>(1,0)  / s;
		z = c_tmp.at<double>(2,0)  / s;
		
		// local frame to image coordinates
		image_x[i] = ( k[0][0] * x + k[0][1] * y + k[0][2] * z ) / z;
		image_y[i]  = ( k[1][0] * x + k[1][1] * y + k[1][2] * z )/ z;  //scaling factor lidar[i][0]  is a constant
	}
	Mat frame1;
	frame.copyTo(frame1);
	frame.copyTo(frame2);
	Mat sure_bg=obstacleEliminate(frame1,0.3,-0.9,30,0);
	//0.3,-0.9
	imshow("sure_bg",sure_bg);
	threshold(sure_bg,sure_bg,1,255,CV_THRESH_BINARY_INV);
	cvtColor(sure_bg,sure_bg,CV_BGR2GRAY);
	Mat sure_fg=obstacleEliminate(frame,0,-0.5,5,1);
	imshow("sure_fg",sure_fg);
	threshold(sure_fg,sure_fg,1,255,CV_THRESH_BINARY_INV);
	cvtColor(sure_fg,sure_fg,CV_BGR2GRAY);
	Mat kernel = Mat::ones(3,3,CV_8UC1);
	erode(sure_fg,sure_fg,kernel);
    erode(sure_fg,sure_fg,kernel);
    erode(sure_fg,sure_fg,kernel);
  
	Mat unknown;
    subtract(sure_bg,sure_fg,unknown);
    Mat markers(frame2.rows,frame2.cols,CV_8UC1);
    int n;
    for(int i=0;i<frame2.rows;i++)
    {
    	for(int j=0;j<frame2.cols;j++)
    	{
    		if(unknown.at<uchar>(i,j) == 255)
    			markers.at<uchar>(i,j) = 0;
    		else if(sure_fg.at<uchar>(i,j) == 255)
    			markers.at<uchar>(i,j) = 255;
    		else if(sure_bg.at<uchar>(i,j) == 0)
    			markers.at<uchar>(i,j) = 128;
    	}
    }
    markers.convertTo(markers,CV_32SC1);
    watershed(frame2,markers);
    markers.convertTo(markers,CV_8UC1);
    for(int i=0;i<frame2.rows;i++)
    {
    	for(int j=0;j<frame2.cols;j++)
    	{
    		if(markers.at<uchar>(i,j) == 255)
    		{
    			frame2.at<Vec3b>(i,j)[0] = (0,0,0);
    			frame2.at<Vec3b>(i,j)[1] = (0,0,0);
    			frame2.at<Vec3b>(i,j)[2] = (0,0,0);
    		}
    	}
    }
    imshow("output",frame2);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	  try 
	  {
	  	frame  = cv_bridge::toCvShare(msg, "bgr8")->image;
	    	cv::waitKey(5);
	  }
	  catch (cv_bridge::Exception& e)
	  {
	    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  }
}