#!/usr/bin/python

import os
import rospy
import pygame.camera
import pygame.image
import datetime
import json
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import move_base_msgs.msg as mb

import actionlib

class task():
    file_path = ""
    target_locations = []
    position = 0

    max_cmd_vel = 1
    min_cmd_vel = .2
    goal_gap = 10

    def __init__(self):
        rospy.init_node('peach_task', anonymous=True)
        #self.file_path = rospy.get_param('/peach_task/task_file')
        self.file_path = "/mnt/waypoints.txt"
        pub = rospy.Publisher('take_picture', String, queue_size=10)
        picture_pause = rospy.Rate(.5) #.5 hz = 2 sec


	cmd_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        cmd_vel_pause = rospy.Rate(50) #.5 hz = 2 sec
        
	tf_list = tf.TransformListener()

        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, "r") as f:
                    self.target_locations = json.load(f)
                print "loaded ", self.target_locations
            except:
                pass

        rospy.Subscriber("/peach_goal", PoseStamped, self.add_target)

        goal_ind = 0;
        while(not rospy.is_shutdown()):
            while(goal_ind < len(self.target_locations)):
	    	trans, rot = tf_list.lookupTransform('/odom', '/base_link', rospy.Time(0))
                print "going to next point"
                goal = mb.MoveBaseGoal();
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = rospy.Time.now();
                x, y, z, w = self.target_locations[goal_ind]

		goal_trans = y
	        rospy.loginfo("Currently at {} need to reach {}".format(trans, goal_trans))
		while abs(trans[0] - goal_trans[0]) > goal_gap:
	    		trans, rot = tf_list.lookupTransform('/odom', '/base_link', rospy.Time(0))


			absspeed = min(max_speed, max(min_speed, 10 * abs(trans[0] - goal_trans[0])))
			speed = absspeed if trans[0] > goal_trans[0] else -absspeed
		        cmd = geometry_msgs.msg.Twist()
		        cmd.linear.x = speed
		        cmd.angular.z = angular
		        turtle_vel.publish(cmd)
	                rospy.loginfo("Currently at {} need to reach {}".format(trans, goal_trans))
	                rospy.loginfo("Sending cmd_vel {}".format(speed))
                    	cmd_vel_pause.sleep()

                if(ac.get_state() == actionlib.GoalStatus.SUCCEEDED):
                    rospy.loginfo("Goal achieved, attempting picture");
                    picture_pause.sleep()
                    pub.publish(str(goal_ind))
                else:
                    rospy.loginfo("Goal failed");

                goal_ind += 1

            picture_pause.sleep()
    
    def add_target(self,data):
        print "new goal {}".format(data)
	(trans,rot) = tf_list.lookupTransform('/odom', '/base_link', rospy.Time(0))
        self.target_locations.append([
            0, 
            trans, 
            0, 
            0])
        with open(self.file_path, "w") as f:
            json.dump(self.target_locations, f)

    def __del__(self):
        with open(self.file_path, "w") as f:
            json.dump(self.target_locations, f)
       

def main():
    task()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
