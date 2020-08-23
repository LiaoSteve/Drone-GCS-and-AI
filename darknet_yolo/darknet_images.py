from ctypes import *
import random
import os
import cv2
import darknet
import argparse

def parser():
    parser = argparse.ArgumentParser(description="YOLO Object Detection")   
    parser.add_argument("--input", default="0",
                        help="webcam or video path")
    parser.add_argument("--weights", default="yolov4.weights",
                        help="yolo weights path") 
    parser.add_argument("--config_file", default="./cfg/yolov4.cfg",
                        help="path to config file")
    parser.add_argument("--data_file", default="./cfg/coco.data",
                        help="path to data file")
    parser.add_argument("--thresh", type=float, default=.25,
                        help="remove detections with confidence below this value")       
    return parser.parse_args()