#!/usr/bin/python

import rospy
import pygame.camera
import pygame.image
import datetime
from std_msgs.msg import String

class camera():
    file_path = ""
    cameras = []


    def __init__(self):
        pygame.camera.init()
        camera_list = ["/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_65CF87FF-video-index0"]
        for i, cam_name in enumerate(camera_list):
            try:
                cam = pygame.camera.Camera(cam_name, (1920, 1080))
                print("found camera {}, size {}".format(i, cam.get_size()))
                cam.start()
                self.cameras.append(cam)
            except:
                pass

        rospy.init_node('peach_camera', anonymous=True)
        self.file_path = rospy.get_param('/peach_camera/file_path')
        rospy.Subscriber("take_picture", String, self.take_picture)
        rospy.spin()
    
    def take_picture(self,data):
        for i, cam in enumerate(self.cameras):
                img = cam.get_image()
                file_name = "{}{}_{}-{}.bmp".format(self.file_path, data.data, i, datetime.datetime.now().strftime("%Y_%m_%d_%H-%M-%S"))
                rospy.loginfo("Camera saving %s", file_name)
                pygame.image.save(img, file_name)

    def __del__(self):
        pygame.camera.quit()

def main():
    camera()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
