from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time
import colorsys
import darknet

def convertBack(x, y, w, h, *size_out):
    global netMain
    size = darknet.network_width(netMain)    
    xmin = int( round( (x - (w / 2))* size_out[0][1] / size ) )
    xmax = int( round( (x + (w / 2))* size_out[0][1] / size ) )
    ymin = int( round( (y - (h / 2))* size_out[0][0] / size ) )
    ymax = int( round( (y + (h / 2))* size_out[0][0] / size ) )
    return xmin, ymin, xmax, ymax


def cvDrawBoxes(detections, img):
    global altNames
    #t  = time.time()    
    num_classes = len(altNames)
    hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))       
    random.seed(68)    
    
    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h), img.shape)
        pt1 = (xmin, ymin)
        pt2 = (xmax, ymax)    
        bbox_color = colors[altNames.index(detection[0].decode())]
        cv2.putText(img,
                    detection[0].decode() +
                    " (" + str(int(detection[1] * 100)) + "%)",
                    (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8,
                    [0, 0, 0], 6)
        cv2.putText(img,
                    detection[0].decode() +
                    " (" + str(int(detection[1] * 100)) + "%)",
                    (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8,
                    [0, 255, 255], 1)    
        cv2.rectangle(img, pt1, pt2, bbox_color, 2) 
    #print(f'time: {time.time()-t}')               
    return img

def YOLO():
    global metaMain, netMain, altNames         
    configPath = "./cfg/my_yolov4.cfg"
    weightPath = "./backup/my_yolov4_best.weights"
    metaPath = "./data/obj.data"
    
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
    # Create an image we reuse for each detect    
    darknet_image = darknet.make_image(darknet.network_width(netMain),
                                    darknet.network_height(netMain),3)                                    
    cur_path = os.getcwd()
    dataset_dir = './data/VOCdevkit/VOC2007/JPEGImages/crawler3/'
    save_dir = './predict_batch_image/trash_crawler3/'
    os.makedirs(save_dir, exist_ok=1)
    images = list()
    for filename in os.listdir(dataset_dir):
        if filename.endswith('jpg'):
            images.append(filename)
    for image in images:        
        frame = cv2.imread(dataset_dir + image)          
        temp = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        temp = cv2.resize(temp, (darknet.network_width(netMain),
                                darknet.network_height(netMain)),
                                interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image, temp.tobytes())

        detections = darknet.detect_image(netMain, metaMain, darknet_image, thresh=0.25)

        frame = cvDrawBoxes(detections, frame)          
        cv2.imwrite(save_dir + 'out_' + image, frame)
        print(f'- [x] save image {image} to {save_dir}')
        '''cv2.imshow('Demo', frame)         
        if cv2.waitKey(0) & 0xff==27:
            continue'''
    print('Well Done')   
   

   
if __name__ == "__main__":
    YOLO()
