#!/usr/bin/python

import rospy
import sys
import cv2
import datetime
from std_msgs.msg import String

class camera():
    file_path = ""
    cameras = []


    def __init__(self):
        pygame.camera.init()
        camera_list = [
          "usb-046d_C922_Pro_Stream_Webcam_725553DF-video-index0",
          "usb-046d_C922_Pro_Stream_Webcam_81B353DF-video-index0",
          "usb-046d_C922_Pro_Stream_Webcam_BE4553DF-video-index0",
          "usb-046d_HD_Pro_Webcam_C920_FAD473DF-video-index0"]

        rospy.init_node('peach_camera', anonymous=True)
        self.file_path = rospy.get_param('/peach_camera/file_path')
        rospy.Subscriber("take_picture", String, self.take_picture)
        rospy.spin()

    def take_picture(self,data):
        for i, cam_name in enumerate(camera_list):
            try:
              cap1 = cv2.VideoCapture("/dev/v4l/by-id/" + cam_name)
              cap1.set(3,1920)
              cap1.set(4,1080)
              ret,img = cap1.read()
              cap1.release()
              file_name = "{}tree{}cam{}date{}.bmp".format(self.file_path, data.data, i, datetime.datetime.now().strftime("%Y_%m_%d_%H-%M-%S"))
              print 1, ret
              if not img is None:
                  rospy.loginfo("Camera saving %s", file_name)
                  cv2.imwrite(filename,img)
              else:
                  rospy.loginfo("Error saving %s", file_name)
            except Exception as e:
                rospy.loginfo("Error in camera script {}".format(str(e)))
                pass


def main():
    camera()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
