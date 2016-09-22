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


    def __init__(self):

        rospy.init_node('peach_task', anonymous=True)
        #self.file_path = rospy.get_param('/peach_task/task_file')
        self.file_path = "waypoints.txt"
        pub = rospy.Publisher('take_picture', String, queue_size=10)
        picture_pause = rospy.Rate(.5) #.5 hz = 2 sec
        
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, "r") as f:
                    self.target_locations = json.load(f)
                print "loaded ", self.target_locations
            except:
                pass

        rospy.Subscriber("/peach_goal", PoseStamped, self.add_target)

        ac = actionlib.SimpleActionClient("move_base", mb.MoveBaseAction)
        while(not ac.wait_for_server(rospy.Duration(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up");

        goal_ind = 0;
        while(not rospy.is_shutdown()):
            while(goal_ind < len(self.target_locations)):
                print "going to next point"
                goal = mb.MoveBaseGoal();
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = rospy.Time.now();
                x, y, z, w = self.target_locations[goal_ind]
                goal.target_pose.pose.position.x = x;
                goal.target_pose.pose.position.y = y;
                goal.target_pose.pose.orientation.z = z;
                goal.target_pose.pose.orientation.w = w;

                rospy.loginfo("Sending goal {},{}".format(x,y));
                ac.send_goal(goal);

                ac.wait_for_result(rospy.Duration(60));

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
        self.target_locations.append([
            data.pose.position.x, 
            data.pose.position.y, 
            data.pose.orientation.z, 
            data.pose.orientation.w])
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
