import sys
import cv2
import time


webcams = [
#"usb-046d_C922_Pro_Stream_Webcam_725553DF-video-index0",
#"usb-046d_C922_Pro_Stream_Webcam_81B353DF-video-index0",
#"usb-046d_C922_Pro_Stream_Webcam_BE4553DF-video-index0",
"usb-046d_HD_Pro_Webcam_C920_FAD473DF-video-index0"]


caps = []
for wc in webcams:
    #cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)I420, framerate=(fraction)24/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
    cap = cv2.VideoCapture("/dev/v4l/by-id/" + wc)
    #cap.set(3,1920)

    #cap.set(4,1080)

    #cap.set(15, -8.0)
    caps.append(cap)

time.sleep(2)

key = cv2.waitKey(10)

count = 0

while True:
    count += 1
    print("start")
    for ind, cap in enumerate(caps):
        ret,img = cap.read()
        print(ind, ret)
        if not img is None:
            print("TT")
            cv2.imshow("2", img)
            print("TTT")
            if key == 32:
                cv2.imwrite('{}-{}.png'.format(count, ind),img)
        key = cv2.waitKey(10)
        if key == 27:
            break
    print("end")

cap.release()
cv2.destroyAllWindows()
