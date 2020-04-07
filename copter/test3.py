import numpy as np
import cv2

capture_usb = cv2.VideoCapture(0)
capture_usb2 = cv2.VideoCapture(1)
stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
while True:            
    read_code2, imgL = capture_usb.read()
    read_code3, imgR = capture_usb2.read()	
    imgL = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)
    disparity = stereo.compute(imgR, imgL)

    cv2.imshow("imgL", imgL)
    cv2.imshow("imgR", imgR)
    cv2.imshow("depth", disparity)			
    if cv2.waitKey(1) == 27:					
        capture_usb.release()	
        capture_usb2.release()			
        break
 
 



