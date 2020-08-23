from ctypes import *
import random
import os
import cv2
import time
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
    parser.add_argument("--out_filename", type=str, default="",
                        help="inference video name. Not saved if empty")   
    parser.add_argument("--dont_show", default=False,
                        help="windown inference display. For headless systems")                        
    parser.add_argument("--ext_output", default=True,
                        help="display bbox coordinates of detected objects")      
    
    return parser.parse_args()


def str2int(video_path):   
    try:
        return int(video_path)
    except ValueError:
        return video_path


def check_arguments_errors(args):
    assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(args.config_file):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
    if not os.path.exists(args.weights):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
    if not os.path.exists(args.data_file):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
    if str2int(args.input) == str and not os.path.exists(args.input):
        raise(ValueError("Invalid video path {}".format(os.path.abspath(args.input))))


def set_saved_video(input_video, output_video, size, fps):
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")    
    video = cv2.VideoWriter(output_video, fourcc, fps, size)
    return video


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
    input_path = str2int(args.input)    

    cap = cv2.VideoCapture(input_path)
    cap.set(3, 640)
    cap.set(4, 480)
    cap_width = cap.get(3)
    cap_hight = cap.get(4)    
    cap_fps = int(cap.get(5))
    video = set_saved_video(cap, args.out_filename, (cap_width, cap_hight), cap_fps)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (darknet_width, darknet_height),
                                   interpolation=cv2.INTER_LINEAR)                
        darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())
        prev_time = time.time()
        detections = darknet.detect_image(network, class_names, darknet_image, thresh=args.thresh)
        fps = int(1/(time.time() - prev_time))
        print(f'fps: {fps}')
        darknet.print_detections(detections, args.ext_output)        
        
        image = darknet.draw_boxes(detections, frame, class_colors, darknet_width)
        
        if args.out_filename is not None:
            video.write(image)
        if not args.dont_show:
            cv2.imshow('Inference', image)
        if cv2.waitKey(fps) == 27:
            cap.release()
            video.release()
            cv2.destroyAllWindows() 
            break
                 
