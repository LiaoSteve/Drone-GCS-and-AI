"""
Author: LiaoSteve
"""
from ctypes import *
import random
import os
import cv2
import darknet
import argparse

sets=[('2007', 'train'), ('2007', 'trainval'), ('2007', 'val'), ('2007', 'test')]

def parser():
    parser = argparse.ArgumentParser(description="YOLO Object Detection") 
    parser.add_argument("--dataset_list", type=str, default="./data/",
                        help="path to your image set ")  

    parser.add_argument("--save_dir", type=str, default="./predict_image/day1/",
                        help="path to save detection images")

    parser.add_argument("--weights", default="./backup/yolov4_final.weights",
                        help="yolo weights path") 

    parser.add_argument("--config_file", default="./cfg/yolov4.cfg",
                        help="path to config file")

    parser.add_argument("--data_file", default="./data/obj.data",
                        help="path to data file")

    parser.add_argument("--thresh", type=float, default=.25,
                        help="remove detections with confidence below this value")    

    return parser.parse_args()


def check_arguments_errors(args):    
    assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(args.config_file):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
    if not os.path.exists(args.weights):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
    if not os.path.exists(args.data_file):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
    if not os.path.exists(args.dataset_list):
        raise(ValueError("Invalid image set file path {}".format(os.path.abspath(args.data_file))))
    os.makedirs(args.save_dir, exist_ok=1)


if __name__ == '__main__':
    args = parser()
    check_arguments_errors(args)
    network, class_names, class_colors = darknet.load_network(
            args.config_file,
            args.data_file,
            args.weights,
            batch_size=1
        )

    darknet_width = darknet.network_width(network)
    darknet_height = darknet.network_height(network)
    darknet_image = darknet.make_image(darknet_width, darknet_height, 3)    
    
    for year, set in sets:
        save_dir = args.save_dir + set +'/'
        os.makedirs(save_dir, exist_ok=1)
        temps = list()      
        images = list()
        f = open(args.dataset_list+year+'_'+set+'.txt')
        for filename in f.readlines():
            filename = filename.strip('\n')
            temps.append(filename)
        f.close()
        for filename in temps:
            if filename.endswith('jpg') or filename.endswith('png')\
                or filename.endswith('jpeg'):
                images.append(filename)
            else:        
                raise RuntimeError(f'notice that {filename} image format are not accepted(.jpg, .png, .jpeg)')
        for image in images:        
            frame = cv2.imread(image)          
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb, (darknet_width, darknet_height),
                                    interpolation=cv2.INTER_LINEAR)
            darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())
            detections = darknet.detect_image(network, class_names, darknet_image, thresh=args.thresh)
            frame = darknet.draw_boxes(detections, frame, class_colors, darknet_width)
            cv2.imwrite(save_dir + image.split('/')[-1], frame)
            print(f'- [x] save image {image} to {save_dir}')
        #print(f'- [OK] Save {len(images)} images done')
        del temps
        del images
