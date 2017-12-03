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
int x_tmp, y_tmp, z_tmp, qx_tmp, qy_tmp, qz_tmp, qw_tmp, roll_tmp, pitch_tmp, yaw_tmp;
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


//trackbar
void on_trackbar_x( int, void* ){
	 c_pos[0] = (float)x_tmp / 100 -1;
	}
void on_trackbar_y( int, void* ){
	 c_pos[1] = (float)y_tmp  / 100 -1;
	}
void on_trackbar_z( int, void* ){
	 c_pos[2] = (float)z_tmp  / 100 -1;
	}
void on_trackbar_qx( int, void* ){
	 c_pos[3] = (float)qx_tmp  / 100 -1;
	}
void on_trackbar_qy( int, void* ){
	 c_pos[4] = (float)qy_tmp / 100 -1;
	}
void on_trackbar_qz( int, void* ){
	 c_pos[5] = (float)qz_tmp / 100 -1;
	}
void on_trackbar_qw( int, void* ){
	 c_pos[6] = (float)qw_tmp / 100 -1;
	}
void on_trackbar_roll( int, void* ){
	 roll = (float)roll_tmp  / 100 - 1;
	}
void on_trackbar_pitch( int, void* ){
	 pitch = (float)pitch_tmp / 100 - 1;
	}
void on_trackbar_yaw( int, void* ){
	 yaw = (float)yaw_tmp / 100 - 1;
	}


cv::Mat frame = Mat::zeros( 200, 200,CV_8UC3);
Mat black1 = Mat::zeros( 200, 200,CV_8UC3);

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


//initializations
double tmp1[4][4] = {{ 1, 0, 0, 0},
					 { 0, 1, 0, 0},
					 { 0, 0, 1, 0},
					 { 0, 0, 0, 0}};

Mat R = Mat(4, 4, CV_64FC1, &tmp1);

double tmp2[4][4] = {{ 1, 0, 0, c_pos[0] },
				 	 { 0, 1, 0, c_pos[1] },
					 { 0, 0, 1, c_pos[2] },
					 { 0, 0, 0, 0        }};

Mat trans = Mat(4, 4, CV_64FC1, &tmp2);

Mat tsf = trans;

double test[4][4];

int main( int argc, char **argv )
{
	//cant perform operations outside of main
	tsf =  tsf + R;

	//trackbar
	namedWindow("Trackbar", 1);
	createTrackbar( "x", "Trackbar", &x_tmp, 200, on_trackbar_x );
	createTrackbar( "y", "Trackbar", &y_tmp, 200, on_trackbar_y);
	createTrackbar( "z", "Trackbar", &z_tmp, 200, on_trackbar_z);
	createTrackbar( "qx", "Trackbar", &qx_tmp, 200, on_trackbar_qx);
	createTrackbar( "qy", "Trackbar", &qy_tmp, 200, on_trackbar_qy);
	createTrackbar( "qz", "Trackbar", &qz_tmp, 200, on_trackbar_qz);
	createTrackbar( "qw", "Trackbar", &qw_tmp, 200, on_trackbar_qw);
	createTrackbar( "roll", "Trackbar", &roll_tmp, 200, on_trackbar_roll);
	createTrackbar( "pitch", "Trackbar", &pitch_tmp, 200, on_trackbar_pitch);
	createTrackbar( "yaw", "Trackbar", &yaw_tmp, 200, on_trackbar_yaw);


	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			test[i][j] = tsf.at<double>(i , j);

	//basic ros stuff
	ros::init(argc, argv, "dataFusion");
	ros::NodeHandle n;
	
	//tf
	tf::TransformListener listener;

	image_transport::ImageTransport it(n);   
	image_transport::Publisher pub = it.advertise("obstacle_free_image", 1);
	
	//scaning
	image_transport::Subscriber sub = it.subscribe("/image", 20, imageCallback);
	ros::Subscriber sub_laser = n.subscribe("scan", 3, laserdata_recieved);

	while(n.ok())
	{
		tf::StampedTransform transform;
		try
		{
			listener.lookupTransform("/laser", "/camera",  
			ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		// Reading Quaternion and updating R and Transform
		c_pos[0] = transform.getOrigin().x();
		c_pos[1] = transform.getOrigin().y();
		c_pos[2] = transform.getOrigin().z();

		//transforms from tf
		double x, y, z, w;
		x = transform.getRotation().x();
		y = transform.getRotation().y();
		z = transform.getRotation().z();
		w = transform.getRotation().w();
		toEulerianAngle(x, y, z, w, roll, pitch, yaw);

		//custom transforms
		on_trackbar_x( x_tmp, 0 );
		on_trackbar_y( y_tmp, 0 );
		on_trackbar_z( z_tmp, 0 );
		// on_trackbar_qx( qx_tmp, 0 );
		// on_trackbar_qy( qy_tmp, 0);
		// on_trackbar_qz( qz_tmp, 0 );
		// on_trackbar_qw( qw_tmp, 0 );
		on_trackbar_roll( roll_tmp, 0 );
		on_trackbar_pitch( pitch_tmp, 0 );
		on_trackbar_yaw( yaw_tmp, 0 );

		toQuaternion(x, y, z, w, roll, pitch, yaw);
		toQuaternion(c_pos[3], c_pos[4], c_pos[5], c_pos[6], roll, pitch, yaw);

		// x = c_pos[5];
		// y = c_pos[4];
		// z = c_pos[3];
		// w = c_pos[6];
		
		// cout << "X: " << c_pos[0] << ", Y: " << c_pos[1] << endl;

		double tmp3[4][4] = {    {     1 - 2*y*y - 2*z*z,        2*x*y - 2*w*z ,         2*x*z + 2*y*w,      0},
								 {     2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,           2*y*z-2*x*w,      0},
								 {     2*x*z - 2*y*w,             2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y,      0},
								 {                    0,                            0,   0,      1}};

		R = Mat(4, 4, CV_64FC1, &tmp3);	
		double tmp4[4][4] = {    {                    0,                    0 ,                      0,      c_pos[0] },
			{                    0,                     0,                      0,      c_pos[1] },
			{                    0,                     0,                      0,      c_pos[2] },
			{                    0,                     0,                      0,                 0} };

		trans = Mat(4, 4, CV_64FC1, &tmp4);
		tsf = trans;
		tsf =  tsf + R;
		
		//publishing
		imshow("With objects1",black1);
		waitKey(30);
		sensor_msgs::ImagePtr msg;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", black1).toImageMsg();
		pub.publish(msg);
		
		ros::spinOnce();
	}

	return 0;
}

void obstacleEliminate(Mat image)
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
			
			
			double tmp[4][1]= {     { lidar[i][1] },
							{         0.1     }, //Also for Anay. Top Heihgt.
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
							{        -0.3  }, //For Anay. Bottom Height
							{ lidar[i][0] },
							{         1     }  };


				pose = Mat(4, 1, CV_64FC1, &tmp2);

				c_tmp = tsf * pose;

				 s = c_tmp.at<double>(3,0);
				 x = c_tmp.at<double>(0,0)  / s;
				 y = c_tmp.at<double>(1,0)  / s;
				 z = c_tmp.at<double>(2,0)  / s;

				int image_y_temp2  = ( k[1][0] * x + k[1][1] * y + k[1][2] * z )/ z;  //scaling factor lidar[i][0]  is constant


			positive[i] = 1; //something here

			for(int j = image_y_temp; j < image_y_temp2; j ++)
				for(int k = ix - 7; k < ix + 7; k++) //width of the angle
					if( k >= 2 && k < black.cols - 2 && j >= 2 && j < black.rows - 2){
						
						blackOut(black, j, k);
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
							{         0.1     },
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
							{        -0.3     },
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
					for(int k = ix  - 22; k < ix - 7; k++) //width of the angle
						if( k >= 2 && k < black.cols - 2 && j >= 2 && j < black.rows - 2)
							blackOut(black, j, k);
				}
			}

			i = i + 1; //switching back original angle
		}
	}
	black.copyTo(black1);
	//imshow("With objects",black1);
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
	image.at<Vec3b>( y, x )[0]  = 0; //color
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
		lidar[i][0] = ranges[i] * cos(  (i+180) * angle_increment + angle_min);
		//x
		lidar[i][1] = - ranges[i] * sin( (i+180) * angle_increment + angle_min ); //x and y coordinates in m



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

	obstacleEliminate(frame);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	  try 
	  {
	  	frame  = cv_bridge::toCvShare(msg, "bgr8")->image;
	    	cv::waitKey(30);
	  }
	  catch (cv_bridge::Exception& e)
	  {
	    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  }
}