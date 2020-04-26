import cv2
import pyrealsense2 as rs
import numpy as np
import threading, time, logging
'''
-----------------------------------
@ Auther : LiaoSteve              
@ Adapted from Tzung-Hsien Huang. 
-----------------------------------'''
class RealSense():
    def __init__(self):
        self.__realsense_log = logging.getLogger(__name__)            
        self.__realsense_log.setLevel(logging.INFO)
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        #self.__config.enable_device('823112061027')
        self.__config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.__config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.__profile = self.__pipeline.start(self.__config)

        self.__depth_sensor = self.__profile.get_device().first_depth_sensor()
        self.__depth_scale  = self.__depth_sensor.get_depth_scale()

        self.__process_frame = []
        self.__frames      = []   

        self.color_frame, self.depth_frame, self.combine_frame = [], [], []       

        self.realsense_isstop = False    

    def realsense_info(self):
        # show device info.    
        time.sleep(1)
        devices = rs.context().query_devices()        
        self.__realsense_log.info('{}'.format(devices[0]))

    def realsense_get_frame(self):
        self.combine_frame = np.hstack((self.color_frame ,self.depth_frame))
        return self.combine_frame    

    def realsense_start(self):
        # Start a thread working for getting the obstacle values as fuzzy inputs.        
        threading.Thread(target=self.operation, daemon=True, args=()).start()       

    def realsense_stop(self):
        # Stop the operation.
        self.realsense_isstop = True                 
        self.__pipeline.stop() 
        self.__realsense_log.info(" >> Stop the RealSense.")

    def filter_setup(self):
        # Spatial filter
        self.__spatial         = rs.spatial_filter()
        self.__spatial.set_option(rs.option.holes_fill, 2) # default: 0
        self.__spatial.set_option(rs.option.filter_smooth_alpha, 1) # default: 0.5, range:[0.25,1]
        self.__spatial.set_option(rs.option.filter_smooth_delta, 50) # default: 20, range:[1, 50]
        # For longer range
        self.__depth2disparity = rs.disparity_transform(True)
        self.__disparity2depth = rs.disparity_transform(False)
        # Hole filling filter
        self.__hole_filling    = rs.hole_filling_filter(mode=2)
        # Threshold Filter
        self.__threshold       = rs.threshold_filter()
        self.__threshold.set_option(rs.option.max_distance, 8) # in meter
        self.__threshold.set_option(rs.option.min_distance, 0.28)

    def imgprocessing(self, frame_in):
        #frame = self.depth2disparity.process(frame_in)
        #frame = self.spatial.process(frame)
        #frame = self.disparity2depth.process(frame)
        frame = self.__hole_filling.process(frame_in)
        self.__process_frame = self.__threshold.process(frame)
        return self.__process_frame

    def operation(self):        
        self.__realsense_log.info(" >> RealSense cam started.")
        self.filter_setup()        
        while (not self.realsense_isstop):
            # Grab data from the device.                        
            self.__frames  = self.__pipeline.wait_for_frames()            
            self.__depth_f = self.__frames.get_depth_frame()
            self.__color_f = self.__frames.get_color_frame()
            if not self.__depth_f or not self.__color_f:
                continue            
            # Filtering
            self.__process_frame = self.imgprocessing(self.__depth_f)                      
            # Show the RGB frame. As for the Depth's one, apply the colormap.
            self.color_frame = np.asanyarray(self.__color_f.get_data())
            self.__depth_image = np.asanyarray(self.__process_frame.get_data())                    
            
            self.depth_frame = cv2.applyColorMap(cv2.convertScaleAbs(self.__depth_image, alpha=0.03), cv2.COLORMAP_JET)
        

if __name__ =='__main__':      
    RS = RealSense()
    RS.realsense_info()
    RS.realsense_start()
    time.sleep(2)
    import timeit             
    while 1:
        #timer = timeit.default_timer()         
        cv2.imshow('realsense',RS.realsense_get_frame())      
        #print('time: {:.4f}'.format(timeit.default_timer()-timer))                
        key = cv2.waitKey(5) & 0xFF # use jetson xavier use waitKey(5)        
        if key == 27:
            RS.realsense_stop()            
            break