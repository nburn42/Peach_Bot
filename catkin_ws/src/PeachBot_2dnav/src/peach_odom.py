#!/usr/bin/python

import rospy
import datetime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion, TransformStamped

import tf

import math

def handle_odom(data):
    global x
    global y
    global th
    global br
    global odom_pub
    global last_time

    current_time = rospy.Time.now();

    vx = data.linear.x
    vy = data.linear.y
    vth = -data.angular.z

    #compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec() * 1000000;
    delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt;
    delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt;
    delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    #since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_about_axis(th, (0,0,1));

    #send the transform
    br.sendTransform((x,y,0), odom_quat, current_time, "base_link", "odom");

    #next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    #set the position
    odom.pose.pose.position = Point(x,y,0.0);
    odom.pose.pose.orientation = Quaternion(odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]);
    

    #set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    #publish the message
    #odom_pub.publish(odom.pose.pose.position, odom.pose.pose.orientation,
    #    odom.header.stamp, odom.header.frame_id, odom.child_frame_id);

    odom_pub.publish(odom)

    last_time = current_time;

if __name__ == '__main__':
    rospy.init_node("odometry_publisher", anonymous=True)
    last_time = rospy.Time.now();
    x = 0.0
    y = 0.0
    th = 0.0
    odom_pub = rospy.Publisher("odom", Odometry)
    br = tf.TransformBroadcaster()
    rospy.Subscriber('/odom_raw',
                     Twist,
                     handle_odom)
    rospy.spin()
