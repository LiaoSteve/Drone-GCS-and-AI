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
    fontScale = 0.5
    num_classes = len(altNames)
    hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))
    bbox_thick = int(0.6 * (img.shape[0] + img.shape[1]) / 600)
    bbox_color = colors[2]
    random.seed(0)
    random.shuffle(colors)
    random.seed(None)

    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h), img.shape)
        pt1 = (xmin, ymin)
        pt2 = (xmax, ymax)    

        bbox_mess = detection[0].decode() + str(round(detection[1],2))
        
        t_size = cv2.getTextSize(bbox_mess, 0, fontScale, thickness=bbox_thick//2)[0]  
        
        cv2.rectangle(img, pt1, pt2, bbox_color, 2) 
        cv2.rectangle(img, pt1, (pt1[0] + t_size[0], pt1[1] - t_size[1] - 3), bbox_color, -1) # fill      
        cv2.putText(img, bbox_mess, (pt1[0], pt1[1]-2), cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale, (0, 0, 0), bbox_thick//2, lineType=cv2.LINE_AA)
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
    dataset_dir = './data/VOCdevkit/VOC2007/JPEGImages/'
    save_dir = './predict_batch_image/trash_crawler1/'
    os.makedirs(save_dir, exist_ok=1)
    images = os.listdir(dataset_dir)
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
