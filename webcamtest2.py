import sys
import cv2
import time


wc = [
#"usb-046d_C922_Pro_Stream_Webcam_725553DF-video-index0",
#"usb-046d_C922_Pro_Stream_Webcam_81B353DF-video-index0",]
"usb-046d_C922_Pro_Stream_Webcam_BE4553DF-video-index0",
"usb-046d_HD_Pro_Webcam_C920_FAD473DF-video-index0"]

key = cv2.waitKey(10)

count = 0

while True:
    cap1 = cv2.VideoCapture("/dev/v4l/by-id/" + wc[0])
    cap1.set(3,1920)
    cap1.set(4,1080)
    ret,img = cap1.read()
    print 1, ret
    if not img is None:
        cv2.imshow("3", img)
        #if key == 32:
            #cv2.imwrite('{}-{}.png'.format(count, ind),img)
    cap1.release()

    cap2 = cv2.VideoCapture("/dev/v4l/by-id/" + wc[1])
    cap2.set(3,1920)
    cap2.set(4,1080)
    print 2, ret
    ret,img = cap2.read()
    if not img is None:
        cv2.imshow("4", img)
        #if key == 32:
            #cv2.imwrite('{}-{}.png'.format(count, ind),img)
    cap2.release()
    key = cv2.waitKey(10)
    if key == 27:
        break

cv2.destroyAllWindows()
