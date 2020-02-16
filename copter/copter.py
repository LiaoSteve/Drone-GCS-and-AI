# -------------------------------- SERVER SENT EVENTS --------------------
from datetime import datetime
import time, json, sys
import cv2
import io
import socket
import struct
import time
import pickle
import zlib
# ------------------------------------- COPTER -----------------------
import threading
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
from math import asin,sin,cos,sqrt,radians
import numpy as np
#------------------------------- -- GLOBAL VARIABLE -------------------------------------------
import globals_variable
globals_variable.globals_vehicle_init()
globals_variable.object_identify_init()
globals_variable.socket_connect_init()

vehicle   = globals_variable.vehicle
Home      = globals_variable.Home
sock      = globals_variable.sock

TRASH_NUM = globals_variable.trash_num
CAP_NUM   = globals_variable.cap_num
#-------------------------------------- YOLOV3 USE ---------------------------------------------
"""
Class definition of YOLO_v3 style detection model on image and video
"""
import cv2
import colorsys
import os
from timeit import default_timer as timer

from keras import backend as K
from keras.models import load_model
from keras.layers import Input
from PIL import Image, ImageFont, ImageDraw

from yolo3.model import yolo_eval, yolo_body, tiny_yolo_body
from yolo3.utils import letterbox_image
import os
from keras.utils import multi_gpu_model

class YOLO(object):
    _defaults = {
        #"model_path": 'model_data/yolo.h5',
        "model_path": 'model_data/trained_weights_final_009.h5',        
        "anchors_path": 'model_data/yolo_anchors_009.txt',
        "classes_path": 'model_data/voc_classes.txt',
        "score" : 0.25,
        "iou" : 0.45,
        "model_image_size" : (416, 416),
        "gpu_num" : 1,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults) # set up default values
        self.__dict__.update(kwargs) # and update with user overrides
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        self.sess = K.get_session()
        self.boxes, self.scores, self.classes = self.generate()

    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names

    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape(-1, 2)

    def generate(self):
        model_path = os.path.expanduser(self.model_path)
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'

        # Load model, or construct model and load weights.
        num_anchors = len(self.anchors)
        num_classes = len(self.class_names)
        is_tiny_version = num_anchors==6 # default setting
        try:
            self.yolo_model = load_model(model_path, compile=False)
        except:
            self.yolo_model = tiny_yolo_body(Input(shape=(None,None,3)), num_anchors//2, num_classes) \
                if is_tiny_version else yolo_body(Input(shape=(None,None,3)), num_anchors//3, num_classes)
            self.yolo_model.load_weights(self.model_path) # make sure model, anchors and classes match
        else:
            assert self.yolo_model.layers[-1].output_shape[-1] == \
                num_anchors/len(self.yolo_model.output) * (num_classes + 5), \
                'Mismatch between model and given anchor and class sizes'

        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))
        np.random.seed(1010)  # Fixed seed for consistent colors across runs.
        np.random.shuffle(self.colors)  # Shuffle colors to decorrelate adjacent classes.
        np.random.seed(None)  # Reset seed to default.

        # Generate output tensor targets for filtered bounding boxes.
        self.input_image_shape = K.placeholder(shape=(2, ))
        if self.gpu_num>=2:
            self.yolo_model = multi_gpu_model(self.yolo_model, gpus=self.gpu_num)
        boxes, scores, classes = yolo_eval(self.yolo_model.output, self.anchors,
                len(self.class_names), self.input_image_shape,
                score_threshold=self.score, iou_threshold=self.iou)
        return boxes, scores, classes

    def detect_image(self, image):
        global vehicle       
        trash_num = 0
        cap_num = 0
        start = timer()
        
        if self.model_image_size != (None, None):
            assert self.model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert self.model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(self.model_image_size)))
        else:
            new_image_size = (image.width - (image.width % 32),
                              image.height - (image.height % 32))
            boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')

        print(image_data.shape)
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })
        print("---------------------------------------")
        print('Found {} boxes for {}'.format(len(out_boxes), 'img'))

        font = ImageFont.truetype(font='font/FiraMono-Medium.otf',
                    size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
        thickness = (image.size[0] + image.size[1]) // 300

        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = self.class_names[c]
            box = out_boxes[i]
            score = out_scores[i]

            label = '{} {:.2f}'.format(predicted_class, score)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label, font)

            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
            #print(label, (left, top), (right, bottom))
            #------------------------------------------------------
            LABEL = label.split()
            if LABEL[0] =='trash':
                trash_num += 1

            if LABEL[0] =='cap':
                cap_num += 1                        
                 
            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])

            # My kingdom for a good redistributable image drawing library.
            for i in range(thickness):
                draw.rectangle(
                    [left + i, top + i, right - i, bottom - i],
                    outline=self.colors[c])
            draw.rectangle(
                [tuple(text_origin), tuple(text_origin + label_size)],
                fill=self.colors[c])
            draw.text(text_origin, label, fill=(0,0,0), font=font)
            del draw
        #-----------------------------------------------------------------
        if trash_num > 0  or cap_num>0:
            cur_loc = vehicle.location.global_relative_frame
            data = {}
            data['channel'] = '00001'            
            data['latitude'] = cur_loc.lat
            data['longitude'] = cur_loc.lon
            data['timestamp'] = str(datetime.now())
            data['trash_num'] = str(trash_num)
            data['cap_num'] = str(cap_num)
            message = json.dumps(data)            
            socket_send_to_MapServer(message)
                        

        end = timer()
        print(end - start,"sec")
        return image,trash_num,cap_num

    def close_session(self):
        self.sess.close()

def detect_video(yolo, video_path, output_path=""):
    global vehicle   
    global TRASH_NUM
    global CAP_NUM
    class CAM:
        def __init__(self, URL):
            self.Frame = []
            self.status = False
            self.isstop = False
            # 攝影機連接。
            self.vid = cv2.VideoCapture(URL, cv2.CAP_DSHOW)
            if not self.vid.isOpened():
                raise IOError("Couldn't open webcam or video")
            self.video_FourCC = int(self.vid.get(cv2.CAP_PROP_FOURCC))
            #self.video_FourCC2    = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_fps = self.vid.get(cv2.CAP_PROP_FPS)
            self.video_size = (int(self.vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        def start(self):
	    # 把程式放進子執行緒，daemon=True 表示該執行緒會隨著主執行緒關閉而關閉。
            print('cam started!')
            threading.Thread(target=self.queryframe, daemon=True, args=()).start()

        def stop(self):
	    # 記得要設計停止無限迴圈的開關。
            self.isstop = True
            print('cam stopped!')
   
        def getframe(self):
        # 當有需要影像時，再回傳最新的影像。
            return self.Frame
        
        def queryframe(self):
            while (not self.isstop):
                self.status, self.Frame = self.vid.read()                
                if self.status == False:
                    break

                #cv2.namedWindow("Real_Time_Camera", cv2.WINDOW_NORMAL)
                '''cv2.imshow('Real_Time_Camera', self.Frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break'''
            self.vid.release()   
            cv2.destroyAllWindows()

    #------------------  Create mycam --------------------------
    #mycam = CAM('20191010.mov')
    while 1:
        try :
            mycam = CAM(0)
            break
        except Exception as e:
            print('cam error',e)
            time.sleep(1)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 10]
    try:
        webcam_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        webcam_socket.settimeout(5)
        webcam_socket.connect(('140.121.130.133',9998))
    except:
        print('Error to connect to webcam_server\n maybe tou should open webcam page or webcam server')
    #----------------------------------------------------
    mycam.start()
    time.sleep(1)
    video_FourCC    = mycam.video_FourCC
    #video_FourCC2    = mycam.video_FourCC2
    video_fps       = mycam.video_fps
    video_size      = mycam.video_size
    isOutput = True if output_path != "" else False

    if isOutput:
        print("!!! TYPE:", type(output_path), type(video_FourCC), type(video_fps), type(video_size))
        out = cv2.VideoWriter(output_path, video_FourCC, video_fps, video_size)

    accum_time = 0
    curr_fps = 0
    fps = "FPS: ??"
    prev_time = timer()

    while True:
        frame = mycam.getframe()  
        result, webcam_frame = cv2.imencode('.jpg', frame, encode_param)        
        data = pickle.dumps(webcam_frame, 0)
        size = len(data)      
        try:          
            webcam_socket.send(struct.pack(">L", size)+  data)
        except:
            print('\n********\nwebcam connection error')
            try:
                print('try to connect to webcam_Server\n')
                webcam_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                webcam_socket.settimeout(0.1)
                webcam_socket.connect(('140.121.130.133',9998))
                print('ok')
            except:
                print('cannot connect to webcam server\n******')
        if frame is None:
            print('TRASH_NUM_final',TRASH_NUM)    
            print('CAP_NUM_final',CAP_NUM)
            break         

        """--------------  Detect_image  ------------"""
        '''image = Image.fromarray(frame)
        image, trash_num, cap_num = yolo.detect_image(image)

        TRASH_NUM = TRASH_NUM + trash_num
        
        CAP_NUM = CAP_NUM + cap_num
        """------------------------------------------"""

        result = np.asarray(image)
        curr_time = timer()
        exec_time = curr_time - prev_time
        prev_time = curr_time
        accum_time = accum_time + exec_time
        curr_fps = curr_fps + 1

        if accum_time > 1:
            accum_time = accum_time - 1
            fps = "FPS: " + str(curr_fps)
            curr_fps = 0

        cv2.putText(result, text=fps, org=(3, 15), fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.40, color=(0, 125, 50), thickness=1)
        #cv2.namedWindow("yolov3_Result", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("yolov3_Result", result)

        print('TRASH_NUM_final',TRASH_NUM)    
        print('CAP_NUM_final',CAP_NUM)
        
        if isOutput:
            out.write(result)
        time.sleep(0.1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    yolo.close_session()
    print('TRASH_NUM_final',TRASH_NUM)    
    print('CAP_NUM_final',CAP_NUM)'''
def socket_send_to_MapServer(message):
    try:
        global sock
        sock.send(message.encode('utf-8'))
    except:
        print('\n*******\nSend data to MapServer error')
        try:
            print('Try connecting to MapServer ...')
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.1)
            sock.connect(('140.121.130.133',9999))
            mark_vehicle_home(HOME)
            waypoints = read_json('data/X_ground.json')
            generate_checkpoint(waypoints)   
        except:
            print('please open drone Map ...')
            pass
    
def YOLO_JOB():
    global vehicle      
    detect_video(YOLO(image = False),"","")
#    ------------------------------ KAFKA FUNCTION ------------------------------------------  
def mark_vehicle_home(HOME):
    data = {}
    data['channel'] = '00004'
    data['latitude'] = HOME.lat
    data['longitude'] = HOME.lon
    message = json.dumps(data)
    socket_send_to_MapServer(message)
def generate_checkpoint(coordinates): 
    """mark the copter waypoints on the map"""
    #global vehicle   
    global sock
    data = {}  
    data['channel'] = '00003'         
    i = 0
    while i < len(coordinates):        
        data['latitude'] = coordinates[i][1]
        data['longitude'] = coordinates[i][0]       
        data['waypoint'] = i+1
        message = json.dumps(data)
        print(message)
        socket_send_to_MapServer(message)
        
        time.sleep(0.2)      
        i+=1    

def read_json(path):
    # READ COORDINATES FROM JSON 
    input_file = open(path)
    json_array = json.load(input_file)
    coordinates = json_array['features'][0]['geometry']['coordinates']
    return coordinates  
#    ------------------------------ COPTER FUNCTION -----------------------------------------   
def get_attributes():
    print (vehicle.location.local_frame)    #NED
    print (vehicle.attitude)
    print ("Velocity: %s" % vehicle.velocity)
    print (vehicle.gps_0)
    print ("Groundspeed: %s" % vehicle.groundspeed)
    print ("Airspeed: %s" % vehicle.airspeed)
    print ( vehicle.gimbal)
    print (vehicle.battery)
    print ("EKF OK?: %s" % vehicle.ekf_ok)
    print ("Last Heartbeat: %s" % vehicle.last_heartbeat)
    print (vehicle.rangefinder)
    print ("Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print ("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print ("Heading: %s" % vehicle.heading)
    print ("Is Armable?: %s" % vehicle.is_armable)
    print ("System status: %s" % vehicle.system_status.state)  

def send_drone_status_to_GCS():
    global vehicle
    global Home
    global sock
    location = vehicle.location.global_relative_frame
    velocity = vehicle.velocity
    attitude = vehicle.attitude
    
    data = {}
    data['channel']  = '00009'                

    data['velocity_N']     = velocity[0]
    data['velocity_E']     = velocity[1]
    data['velocity_Down']  = velocity[2]

    data['latitude']       = location.lat
    data['longitude']      = location.lon    
    data['altitude']       = location.alt

    data['yaw']            = attitude.yaw
    data['roll']           = attitude.roll
    data['pitch']          = attitude.pitch

    data['groundspeed']    = vehicle.groundspeed 
    data['mode']           = vehicle.mode.name        
    data['dist_to_home']   = haversine( pos1 = location, pos2_lon = Home.lon , pos2_lat = Home.lat)
    data['heading']        = vehicle.heading
    data['gps_status']     = str(vehicle.gps_0)
    data['battery']        = str(vehicle.battery)
    message = json.dumps(data)    
    socket_send_to_MapServer(message)
    
def arm_and_takeoff(aTargetAltitude):
    # Arms vehicle and fly to aTargetAltitude.
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        send_drone_status_to_GCS()  
        cur_pos = send_current_mark()      
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)              
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
def send_current_mark():
    global sock
    data = {}
    cur_pos = vehicle.location.global_relative_frame    
    data['channel'] = '00002'
    data['latitude'] = cur_pos.lat
    data['longitude'] = cur_pos.lon
    message = json.dumps(data)
    socket_send_to_MapServer(message)   
    return cur_pos
def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
def set_roi(location):
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

    For more information see: 
    http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
    """
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing
def goto(target, gotoFunction = vehicle.simple_goto):
    currentLocation = vehicle.location.global_relative_frame
    targetDistance = haversine(currentLocation,target.lon,target.lat)

    print("[ targetDistance ]:",targetDistance," (m)")
    gotoFunction(location=target ,groundspeed=None)
    
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
                
        cur_pos = send_current_mark()
        send_drone_status_to_GCS()
        remainingDistance = haversine(cur_pos,target.lon,target.lat)
        
        vehicle.groundspeed = 1
        #--  error --
        torlerance = 0.08
        if remainingDistance<0.9:
            torlerance = 1
        if remainingDistance<0.31:
            torlerance = 5
        torlerance_dist = remainingDistance * torlerance                                  
      
        print("[Dist to target]:{:.2f}(m),[Speed]:{:.2f}(m/s),[tolerance dist]:{:.2f}(m)".format(remainingDistance,vehicle.groundspeed,torlerance_dist))    

        if remainingDistance<=torlerance_dist: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(1) 
def haversine(pos1,pos2_lon,pos2_lat):
    """
    Get the distance between two positions (note that inputs are in decimal degree)
    pos1 : current point
    pos2 : Reference point 
    """
    lon1, lat1, lon2, lat2 = map(radians, [pos1.lon, pos1.lat, pos2_lon, pos2_lat])
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6378137.0 #-- Radius of "spherical" earth in meter
    return c * r   
def COPTER_JOB():   
    global vehicle    
    try:
        arm_and_takeoff(aTargetAltitude=8)        
        for i in range(len(waypoints)): 
            print("------------------------------------------------------")      
            print('Go to waypoints:',i+1)
            print("[lon,lat]: [",waypoints[i][0],waypoints[i][1],"]")  #[lon ,lat]) 
            print("------------------------------------------------------")     
            target = LocationGlobalRelative(lat=waypoints[i][1],lon=waypoints[i][0],alt=8)
            time.sleep(1)   
            goto(target)  
        print("Mission Completed")
        #print("Setting LAND mode...")
        #vehicle.mode = VehicleMode("LAND")
        print("Setting RTL mode...")
        vehicle.groundspeed = 1
        vehicle.mode = VehicleMode("RTL")                           
        print('TRASH_NUM:',TRASH_NUM)
        print('CAP_NUM:',CAP_NUM)
        while(1):            
            #time.sleep(1)
            cur_pos = send_current_mark()
            send_drone_status_to_GCS()
            print('===================== UAV alive ======================')
            time.sleep(1)
            
        print("Close vehicle object")
        vehicle.close()     
    except Exception as e: 
        print("exception:",e)
        vehicle.close() 
        print(e)     
    finally :
        vehicle.close() 
        sys.exit(0)
def main():
    global vehicle
    t1 = threading.Thread(target=COPTER_JOB)
    t2 = threading.Thread(target=YOLO_JOB)

    t2.start()
    t1.start()
        
#-------------------------------------- MAIN -------------------------------------------
if __name__ == '__main__':               
    HOME = vehicle.location.global_relative_frame
    print('HOME:',HOME)
    time.sleep(1)
    mark_vehicle_home(HOME)
    waypoints = read_json('data/X_ground.json')
    generate_checkpoint(waypoints)          
    print("\n------------- States -----------------")
    get_attributes()    
    main()



