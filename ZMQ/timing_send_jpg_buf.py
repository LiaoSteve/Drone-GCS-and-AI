import sys
import socket
import time
import traceback
import cv2
import imagezmq
sender = imagezmq.ImageSender(connect_to='tcp://140.121.130.133:5555')
name = socket.gethostname() 
cap = cv2.VideoCapture(0)
time.sleep(2.0)  # allow camera sensor to warm up
jpeg_quality = 95  # 0 to 100, higher is better quality, 95 is cv2 default
try:
    while True:  # send images as stream until Ctrl-C
        ret, image = cap.read()              
        if not ret: continue
        ret_code, jpg_buffer = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
        reply_from_server = sender.send_jpg(name, jpg_buffer)        
except (KeyboardInterrupt, SystemExit):
    pass  # Ctrl-C was pressed to end program
except Exception as ex:
    print('Python error with no Exception handler:')
    print('Traceback error:', ex)
    traceback.print_exc()
finally:    
    cap.close()  # stop the camera thread
    sender.close()  # close the ZMQ socket and context
    sys.exit()
