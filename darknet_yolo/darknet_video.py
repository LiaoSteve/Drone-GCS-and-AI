from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time
import darknet

def convertBack(x, y, w, h):
    xmin = int( round( (x - (w / 2))* 640 / 416 ) )
    xmax = int( round( (x + (w / 2))* 640 / 416 ) )
    ymin = int( round( (y - (h / 2))* 480 / 416 ) )
    ymax = int( round( (y + (h / 2))* 480 / 416 ) )
    return xmin, ymin, xmax, ymax


def cvDrawBoxes(detections, img):
    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h))
        pt1 = (xmin, ymin)
        pt2 = (xmax, ymax)        
        cv2.rectangle(img, pt1, pt2, (0, 255, 255), 1)      
        cv2.putText(img,
                    detection[0].decode() +
                    " (" + str(int(detection[1] * 100)) + "%)",
                    (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5,
                    [0, 0, 0], 6)
        cv2.putText(img,
                    detection[0].decode() +
                    " (" + str(int(detection[1] * 100)) + "%)",
                    (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5,
                    [0, 255, 255], 1)            
    return img

def YOLO():
    global metaMain, netMain, altNames
    configPath = "./cfg/yolov4.cfg"
    weightPath = "./yolov4.weights"
    metaPath = "./cfg/coco.data"
    
    netMain = darknet.load_net_custom(configPath.encode(
            "ascii"), weightPath.encode("ascii"), 0, 1)  # batch size = 1   
    metaMain = darknet.load_meta(metaPath.encode("ascii"))    
    try:
        with open(metaPath) as metaFH:
            metaContents = metaFH.read()
            import re
            match = re.search("names *= *(.*)$", metaContents,
                                re.IGNORECASE | re.MULTILINE)
            if match:
                result = match.group(1)
            else:
                result = None
            try:
                if os.path.exists(result):
                    with open(result) as namesFH:
                        namesList = namesFH.read().strip().split("\n")
                        altNames = [x.strip() for x in namesList]
            except TypeError:
                pass
    except Exception:
        pass
    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture("girls1.mp4")
    cap.set(3, 640)
    cap.set(4, 480)
    out = cv2.VideoWriter(
        "output.avi", cv2.VideoWriter_fourcc(*"MJPG"), 10.0,
        (640, 480))
    print("Starting the YOLO loop...")

    # Create an image we reuse for each detect
    darknet_image = darknet.make_image(darknet.network_width(netMain),
                                    darknet.network_height(netMain),3)
    while True:
        prev_time = time.time()
        ret, frame_read = cap.read()
        if not ret: continue 
        frame_rgb = cv2.cvtColor(frame_read, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb,
                                   (darknet.network_width(netMain),
                                    darknet.network_height(netMain)),
                                   interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image,frame_resized.tobytes())

        detections = darknet.detect_image(netMain, metaMain, darknet_image, thresh=0.25)

        image = cvDrawBoxes(detections, frame_read)        
        
        print(f'FPS: {1/(time.time()-prev_time)}')
        cv2.imshow('Demo', image)
        if cv2.waitKey(1) & 0xff == 27:
            cap.release()
            out.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    YOLO()
