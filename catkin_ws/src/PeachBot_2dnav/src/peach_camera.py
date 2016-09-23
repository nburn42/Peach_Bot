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
        for i, cam_ind in enumerate(pygame.camera.list_cameras()):
            if i == 0:
                #skip zed, not doing so causes problems
                continue
            cam = pygame.camera.Camera(cam_ind)
            cam.start()
            self.cameras.append(cam)

        print rospy.get_param_names()

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
