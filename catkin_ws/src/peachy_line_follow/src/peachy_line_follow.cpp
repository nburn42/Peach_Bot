#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <stdlib.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

double linear, angular;
double l_scale, a_scale;
double left_bias, right_bias;
int hll, hhl, hlh, hhh, l1, l2, l3;
int tag_a_color_low, tag_a_color_high, 
	tag_b_color_low, tag_b_color_high;
double speed;
bool tag_goal_a;
int delay_count  = 99;


ros::Publisher cmd_pub;
ros::Publisher task_pub;
image_transport::Subscriber camera_sub;
image_transport::Publisher output_pub;
int loop_count = 0;
int tag_count = 0;

std_msgs::String task_msg;

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
	//if (loop_count++ % 10 != 0) {
	//	return;
	//}
	
	delay_count++;	

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	geometry_msgs::Twist velMsg;
	vector<Vec4i> lines;
	int i, c, d;
	float c1[50];
	float m, angle;
	float buf;
	float m1;
	float dis;
	int k = 0, k1 = 0;
	int count = 0;

	float xv;
	float yv;
	int vc = 0;
	float xvan = 0, yvan = 0;
	static float xvan1 = 0, yvan1 = 0;
	float num = 0;
	static float count1 = 0;
	float dv;
	float vxx, vyy;

	Mat out1;
	Mat gray_out;
	Mat canny_out;
	Mat gray_out1;
	Mat canny_out1;
	Mat canny_out2;

	Mat gray_out2;
	vector<Mat> hsv_channels;
	Mat hsv_out;
	Mat red_out;
	Mat red_out1;
	Mat red_out2;
	Mat tag_out;

	out1 = cv_ptr->image;
	//GaussianBlur(out1, out1, Size(3, 3), 0,0);
	
	rectangle(out1, Point(0,0), Point(640, 350), Scalar(0, 0, 0), -1, 8);
	
	cvtColor(out1, hsv_out, CV_BGR2HSV);
	cv::split(hsv_out, hsv_channels);
	hsv_out = hsv_channels[0];
	//inRange(red_out, hhl, hhh, red_out);
	inRange(hsv_out, hll, hlh, red_out1);
	inRange(hsv_out, hhl, hhh, red_out2);
	red_out = red_out1 | red_out2;
	red_out = (red_out & (hsv_channels[1] > 50)) & (hsv_channels[2] > 60);

	if(tag_goal_a) {
		inRange(hsv_out, tag_a_color_low, tag_a_color_high, tag_out);
	} else {
		inRange(hsv_out, tag_b_color_low, tag_b_color_high, tag_out);
	}
	tag_out = (tag_out & (hsv_channels[1] > 15)) & (hsv_channels[2] > 30);

	erode(red_out, red_out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate(red_out, red_out, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)) ); 

	erode(tag_out, tag_out, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)) );
	dilate(tag_out, tag_out, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)) ); 
	//cvtColor(out1, gray_out1, CV_BGR2GRAY);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<vector<Point> > tag_contours;
	vector<Vec4i> tag_hierarchy;

	findContours( red_out, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	findContours( tag_out, tag_contours, tag_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	Scalar color1 = Scalar( 0, 255, 0 );
	Scalar color2 = Scalar( 255, 0, 0 );
	Scalar color3 = Scalar( 0, 0, 255 );
	double mean_x = 0;
	double mean_y = 0;
	double total_area = 0;

	for( int i = 0; i < contours.size(); i++ )
	{
		drawContours( red_out, contours, i, color2, 2, 8, hierarchy, 0, Point() );
		drawContours( out1, contours, i, color1, 2, 8, hierarchy, 0, Point() );
		Moments con_mom = moments(contours[i], false);
		double area = contourArea(contours[i]);
		double x = con_mom.m10/con_mom.m00;
		double y = con_mom.m01/con_mom.m00;	
		//printf("Moment x %f y %f \n", x, y);
		circle(red_out, Point(x,y), area/10, color2, 5);
		circle(out1, Point(x,y), area/10, color2, 5);
		
		total_area += area;
		double percent = area/total_area;
		mean_x = ((1-percent) * mean_x) + (percent * x);
		mean_y = ((1-percent) * mean_y) + (percent * y);
	}

	
	circle(red_out, Point(mean_x,mean_y), total_area/50, color1, 5);
	circle(out1, Point(mean_x,mean_y), total_area/50, color1, 5);
	
	for( int i = 0; i < tag_contours.size() && delay_count > 4; i++ )
	{
		drawContours( out1, tag_contours, i, color3, 2, 8, tag_hierarchy, 0, Point() );
		double area = contourArea(tag_contours[i]);
		printf("tag %f\n", area);

		if(area > 50) {
			if (tag_goal_a) {
				printf("FOUND A\n");
			} else {
				printf("FOUND B\n");
			}
			std::ostringstream s;
			s << tag_count++;
			std::string tag_as_string(s.str());
			task_msg.data = tag_as_string;
			task_pub.publish(task_msg);
			tag_goal_a = !tag_goal_a;
			delay_count = 0;
			break;
		}
	}

	velMsg.angular.z = 0;
	if (total_area > 150) {
		if (speed < 1.8) {
			speed += 0.3;
		} else {
			speed = 1.8;
		}
		
		// 640 across
		// max range 0 - 640
		// result -0.5 - 0.5
		velMsg.angular.z =  (0.5 - (mean_x/640)) * 2;
	} else {
		if (speed > 0) {
                        speed -= 0.4;
                } else {
			speed = 0;
		}
	}
	velMsg.linear.x = speed;	

	printf("x: %f z: %f size:%f\n", velMsg.linear.x, velMsg.angular.z, total_area);
	cmd_pub.publish(velMsg);

	/*
	HoughLinesP(red_out, lines, 1,
			CV_PI / 360, l1, l2, l3);

	for (i = 0; i < lines.size(); i++) {

		Point pt1(lines[i][0], lines[i][1]);
		Point pt2(lines[i][2], lines[i][3]);
		
		float theta = atan2(lines[i][0] - lines[i][2], lines[i][1] - lines[i][3]) * 180 / 3.14159265;
		float length = sqrt(pow(lines[i][0] - lines[i][2], 2) + pow(lines[i][1] - lines[i][3], 2));

		if (length < 30) {
			continue;
		}
		printf("LEN : %f ", length);

		line(red_out, pt1, pt2, Scalar(0, 255, 0), 1, 8);
		line(out1, pt1, pt2, Scalar(0, 255, 0), 1, 8);

		buf = (lines[i][0] - lines[i][2]) / 2;
		//printf(" buf: %f", buf);
		printf(" theat: %f", theta);
		velMsg.linear.z = buf;
		velMsg.linear.y = theta;

		if (abs(buf) <= left_bias) { // turn left
			velMsg.angular.z = angular * a_scale;
		}
		else if (abs(buf) >= right_bias) { // turn right
			velMsg.angular.z = -angular * a_scale;
		}
		else {
			velMsg.linear.x = linear * l_scale;
		}


		printf("\nX::Y::X1::Y1::%d:%d:%d:%d", lines[i][0], lines[i][1], lines[i][2], lines[i][3]);

		cmd_pub.publish(velMsg);
	}
	*/

	// Added Display
	//namedWindow( "out1", 1 );
	//imshow( "out1", red_out );
	//namedWindow( "out2", 2 );
	//imshow( "out2", out1 );

	waitKey(1);
	//cvShowImage("GRAY_OUT", gray_out);
	//cvShowImage("CANNY_OUT", canny_out);
	//cvShowImage("CANNY_OUT1", canny_out1);
	//cvShowImage("gray_out2", gray_out2);
	//cv_ptr->image = red_out;
	cv_ptr->image = out1;
	output_pub.publish(cv_ptr->toImageMsg());
}


int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "peachy_line_follow");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;
	image_transport::ImageTransport it = image_transport::ImageTransport(n);

	linear = 0.05;
	angular = 0.0;
	l_scale = 1.0;
	a_scale = 1.0;
	left_bias = 150;
	right_bias = 450;
	hll = 150;
	hlh = 179;
	hhl = 0;
	hhh = 15;
	l1 = 100;
	l2 = 30;
	l3 = 1;

	tag_b_color_low =  117;
	tag_b_color_high =  150;
	tag_a_color_low = 110;
	tag_a_color_high = 117;
	tag_goal_a = true;

	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
	task_pub = n.advertise <std_msgs::String> ("take_picture", 10);
	cmd_pub = n.advertise <geometry_msgs::Twist> ("key_vel", 10);
	camera_sub = it.subscribe(
	"/usb_cam/image_raw", 5,
	&imageCb);
	output_pub = it.advertise("/arcv/Image", 1);

	ros::Rate loop_rate(50);

	while (ros::ok())
	{
	//ROS_INFO("loop");


	ros::spinOnce();

	//loop_rate.sleep();
	}


	return 0;
}
