import cv2
from django.shortcuts import render
from django.http import HttpResponse

import time, sys
from django.http import StreamingHttpResponse
from django.utils.timezone import now

class VideoCamera():
    def __init__(self,id):
        # Using OpenCV to capture from device 0. If you have trouble capturing
        # from a webcam, comment the line below out and use a video file
        # instead.
        try:
            self.video = cv2.VideoCapture(id)
        except Exception as e:
            print(e)
            sys.exit(0)
        # If you decide to use video.mp4, you must have this file in the folder
        # as the main.py.
        # self.video = cv2.VideoCapture('video.mp4')
    
    def __del__(self):
        self.video.release()                  

def gen(camera):
    camera = VideoCamera(0)
    while True:
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        pre_time = time.time()
        success, image = camera.video.read()     
        now = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        cv2.putText(image, f'{now}', (50,50), cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255), 1, cv2.LINE_AA)   
        time.sleep(0.1)
        fps = round(1 / (time.time() - pre_time), 2)          
        cv2.putText(image, f'FPS: {fps}', (50,90), cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255), 1, cv2.LINE_AA)      

        ret, jpeg = cv2.imencode('.jpg', image)         
        frame = jpeg.tobytes() 
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

