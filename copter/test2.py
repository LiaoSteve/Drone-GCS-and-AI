import cv2

capture = cv2.VideoCapture(0)
capture_usb = cv2.VideoCapture(1)
capture_usb2 = cv2.VideoCapture(2)
while 1:
	if capture.isOpened():
		if capture_usb.isOpened():			
			while True:
				read_code, frame = capture.read()
				read_code2, frame2 = capture_usb.read()
				read_code3, frame3 = capture_usb2.read()
				if not read_code or not read_code2 :
					break
				cv2.imshow("screen_title", frame)
				cv2.imshow("screen_title_usb", frame2)
				cv2.imshow("screen_title_usb2", frame3)			
				if cv2.waitKey(1) == 27:					
					capture_usb.release()				
					break
# 释放资源      
capture.release()
cv2.destroyWindow("screen_title") 
