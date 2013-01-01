/*
 * Copyright (c) 2012, Tang Tiong Yew
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
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
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <dynamic_reconfigure/server.h>
#include <eddiebot_line_follower/paramsConfig.h>
/*here is a simple program which demonstrates the use of ros and opencv to do image manipulations on video streams given out as image topics from the monocular vision
 of robots,here the device used is a ardrone(quad-rotor).*/
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image window";

float prevVelocity_angular, prevVelocity_linear, newVelocity_angular,
		newVelocity_linear;
float derive_angular, derive_linear, dt = 0.5;
float horizontalcount;

dynamic_reconfigure::Server<eddiebot_line_follower::paramsConfig> *             dynamic_reconfigure_server;
dynamic_reconfigure::Server<eddiebot_line_follower::paramsConfig>::CallbackType dynamic_reconfigure_callback;


class Follow {
	ros::NodeHandle nh_;
	ros::Publisher pub;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_; 
	std_msgs::String msg;

	double linear_, angular_;
	double l_scale_, a_scale_;
	double left_, right_;
	int hll_, hhl_, hlh_, hhh_, l1_, l2_, l3_;


public:
	


	Follow() :
		linear_(0.05),
		angular_(0.05),
		l_scale_(1.0),
		a_scale_(1.0),
		left_(150),
		right_(450),
		hll_(0),
		hhl_(16),
		hlh_(165),
		hhh_(179),
		l1_(50),
		l2_(50),
		l3_(50),
		it_(nh_) {
			pub = nh_.advertise <geometry_msgs::Twist> ("cmd_vel", 1);
			image_sub_ = it_.subscribe("/camera/left/image_rect_color", 1,
					&Follow::imageCb, this);
			image_pub_ = it_.advertise("/arcv/Image", 1);

			dynamic_reconfigure::Server<eddiebot_line_follower::paramsConfig> srv;
			dynamic_reconfigure::Server<eddiebot_line_follower::paramsConfig>::CallbackType f;
			f = boost::bind(&Follow::reconfigCB, this, _1, _2);
			srv.setCallback(f);

			// Initialize Parameters
			nh_.param("scale_angular", a_scale_, a_scale_);
			nh_.param("scale_linear", l_scale_, l_scale_);
			nh_.param("angular", angular_, angular_);
			nh_.param("linear", linear_, linear_);
			nh_.param("left", left_, left_);
			nh_.param("right", right_, right_);
			nh_.param("hll", linear_, linear_);
			nh_.param("hlh", left_, left_);
			nh_.param("hhl", right_, right_);
			nh_.param("hhh", linear_, linear_);
			nh_.param("l1", l1_, l1_);
			nh_.param("l2", l2_, l2_);
			nh_.param("l3", l3_, l3_);

	}

	~Follow() {
		destroyWindow(WINDOW);
	}

	void reconfigCB(eddiebot_line_follower::paramsConfig &config, uint32_t level)
	{
	  //ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f",
		//	   config.speed_lim_v, config.speed_lim_w, config.accel_lim_v, config.accel_lim_w, config.decel_factor, config.speed_scale, config.rotation_scale);

		linear_ = config.scale_linear;
		angular_ = config.scale_angular;
		l_scale_ = config.angular;
		a_scale_ = config.linear;
		left_ = config.left;
		right_ = config.right;
		hll_ = config.hll;
		hhl_ = config.hlh;
		hlh_ = config.hhl;
		hhh_ = config.hhh;
		l1_ = config.l1;
		l2_ = config.l2;
		l3_ = config.l3;

	}


	void imageCb(const sensor_msgs::ImageConstPtr& msg) {

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

		//Rect roi_rect = Rect(0,0,640,480);

		/* ROI data pointer points to a location in the same memory as img. i.e.
		 No separate memory is created for roi data */

		//Mat complement;
		//bitwise_not(cv_ptr->image,complement);
		//complement.copyTo(cv_ptr->image);


		Mat out1;
		Mat gray_out;
		Mat canny_out;
		Mat gray_out1;
// = cvCreateImage(cvGetSize(cv_ptr->image), IPL_DEPTH_8U, 3);
		Mat canny_out1;
// = cvCreateImage(cvGetSize(cv_ptr->image), IPL_DEPTH_8U, 1);
		Mat canny_out2;
// = cvCreateImage(cvGetSize(cv_ptr->image), IPL_DEPTH_8U, 1);

		Mat gray_out2;
// = cvCreateImage(cvGetSize(cv_ptr->image), IPL_DEPTH_8U, 1);
		vector<Mat> hsv_channels;
		Mat hsv_out;
// = cvCreateImage(cvGetSize(cv_ptr->image), IPL_DEPTH_8U, 1);
		Mat red_out;
// = cvCreateImage(cvGetSize(cv_ptr->image), IPL_DEPTH_8U, 1);
		Mat red_out1;
// = cvCreateImage(cvGetSize(cv_ptr->image), IPL_DEPTH_8U, 1);
		Mat red_out2;
// = cvCreateImage(cvGetSize(cv_ptr->image), IPL_DEPTH_8U, 1);

		GaussianBlur(cv_ptr->image, out1, Size(3, 3), 0,0);

		cvtColor(out1, hsv_out, CV_BGR2HSV);
		cv::split(hsv_out, hsv_channels);
		red_out = hsv_channels[0];
		inRange(red_out, hll_, hlh_, red_out1);
		inRange(red_out, hhl_, hhh_, red_out2);
		red_out = red_out1 | red_out2;
		red_out = (red_out & (hsv_channels[1] > 50)) & (hsv_channels[2] > 50);
		erode(red_out, red_out, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)) );
		dilate(red_out, red_out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		cvtColor(out1, gray_out1, CV_BGR2GRAY);

		//bitwise_and(gray_out1, gray_out1, gray_out1, hsv_out);
		
		//bitwise_not(gray_out1,gray_out2);
		//Canny(gray_out2, canny_out, 80, 125, 7);
		//cvtColor(canny_out, gray_out1, CV_GRAY2BGR);

//		//////////////// Color Filtering //////////////
//
//		//IplImage *imgHsv,*imgResult;
//		int mR_val=255,mG_val=128,mB_val=64,MAR_val=128,MAG_val=0,MAB_val=0;//default green .ctrl BLUE to find color
//
//
//		IplImage* test=cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
//		IplImage* imgResult=cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
//		IplImage* imgHsv=cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
//
//		cvCvtColor( test, imgHsv, CV_BGR2HSV);//convert the color space
//		CvScalar min_color = CV_RGB(mR_val,mG_val,mB_val);
//		CvScalar max_color = CV_RGB(MAR_val,MAG_val,MAB_val);
//		cvInRangeS(imgHsv, min_color,max_color, imgResult);//search for the color in image
//		//////////////// Color Filtering //////////////

		
		HoughLinesP(red_out, lines, 1,
				CV_PI / 180, l1_, l2_, l3_);
		for (i = 0; i < lines.size(); i++) {

			//float rho = lines[i][0];
			//float theta = lines[i][1];
			//double a = cos(theta), b = sin(theta);
			//double x0 = a*rho, y0 = b*rho;
			Point pt1(lines[i][0], lines[i][1]);
			Point pt2(lines[i][2], lines[i][3]);
			line(red_out, pt1, pt2, Scalar(0, 255, 0), 1, 8);
			line(out1, pt1, pt2, Scalar(0, 255, 0), 1, 8);
			//xv = line[0].x - line[1].x;
			//yv = line[0].y - line[1].y;
//					velMsg.linear = atan2(xv, yv) * 180 / 3.14159265;
//					angle = velMsg.linear;
//					printf("test");
//					if (velMsg.linear < -90) {
//						printf("one");
//						velMsg.linear = velMsg.linear + 180;
//					}
//					buf = (line[0].x + line[1].x) / 2;
//
//					if (abs(85 - buf) <= 15) {
//						velMsg.angular = 0;
//						printf("two");
//					} else {
//						velMsg.angular = (85 - (line[0].x + line[1].x) / 2);
//						printf("three");
//					}

//					if (abs(velMsg.angular) > 50) {
//						velMsg.angular = 0;
//						printf("four");
//					}
// Modified
			float theta = atan2(lines[i][0] - lines[i][2], lines[i][1] - lines[i][3]) * 180 / 3.14159265;


			buf = (lines[i][0] - lines[i][2]) / 2;
			printf("buf: %f", buf);
			printf("theat: %f", theta);
			velMsg.linear.z = buf;
			velMsg.linear.y = theta;

			if (abs(buf) <= left_) { // turn left
				velMsg.angular.z = angular_ * a_scale_;
			}
			else if (abs(buf) >= right_) { // turn right
				velMsg.angular.z = -angular_ * a_scale_;
			}
			else {
				velMsg.linear.x = linear_ * l_scale_;
			}


			printf("\nX::Y::X1::Y1::%d:%d:%d:%d", lines[i][0], lines[i][1], lines[i][2], lines[i][3]);

			pub.publish(velMsg);

		}
//		cvShowImage("OUT1", out1);   //lines projected onto the real picture
//		cvShowImage("GRAY_OUT1", gray_out1);

		// Added Display
		namedWindow( "out1", 1 );
    		imshow( "out1", red_out );
		namedWindow( "out2", 2 );
    		imshow( "out2", out1 );

		waitKey(1);
		//cvShowImage("GRAY_OUT", gray_out);
		//cvShowImage("CANNY_OUT", canny_out);
		//cvShowImage("CANNY_OUT1", canny_out1);
		//cvShowImage("gray_out2", gray_out2);
		cv_ptr->image = red_out;
		image_pub_.publish(cv_ptr->toImageMsg());

	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "eddiebot_line_follower_node");
	ROS_INFO("Starting to spin...");
	
	Follow ic;
	ros::spin();
	return 0;
}
