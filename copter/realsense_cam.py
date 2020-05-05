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
        # Realsense Logger
        self.__realsense_log = logging.getLogger(__name__)            
        self.__realsense_log.setLevel(logging.INFO)
        # Realsese cam param.       
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        #self.__config.enable_device('823112061027')
        self.__config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.__config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.__profile = self.__pipeline.start(self.__config)
        self.__depth_sensor = self.__profile.get_device().first_depth_sensor()
        self.__depth_scale  = self.__depth_sensor.get_depth_scale()   
        # Create an align object       
        self.__align = rs.align(rs.stream.color)           
        # Realsense cam frame
        self.color_frame, self.depth_frame = None, None  
        self.realsense_isstop = False    
        # Sense obstacle param.
        self.h, self.w = (480, 640)
        self.eq_w = int(self.w/5)
        self.center_L = [int(self.w/8) * 2, int(self.h/2)]
        self.center_M = [int(self.w/8) * 4, int(self.h/2)]
        self.center_R = [int(self.w/8) * 6, int(self.h/2)]
        self.shift = 120  
        # Sense obstacle ROI      
        self.LBox = [((self.center_L[1]-self.shift), (self.center_L[0]-self.eq_w)), ((self.center_L[1]+self.shift), (self.center_L[0]+self.eq_w))]
        self.MBox = [((self.center_M[1]-self.shift), (self.center_M[0]-self.eq_w)), ((self.center_M[1]+self.shift), (self.center_M[0]+self.eq_w))]
        self.RBox = [((self.center_R[1]-self.shift), (self.center_R[0]-self.eq_w)), ((self.center_R[1]+self.shift), (self.center_R[0]+self.eq_w))]

    def realsense_info(self):
        # Show device info.    
        time.sleep(1)
        devices = rs.context().query_devices()        
        self.__realsense_log.info('{}'.format(devices[0]))

    def realsense_get_frame(self):
        self.combine_frame = np.hstack((self.color_frame ,self.depth_colormap))
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
        frame = self.__depth2disparity.process(frame_in)
        frame = self.__spatial.process(frame)
        frame = self.__disparity2depth.process(frame)
        frame = self.__hole_filling.process(frame)
        frame = self.__threshold.process(frame)
        return frame

    def operation(self):        
        self.__realsense_log.info(" >> RealSense cam started.")
        self.filter_setup()        
        while (not self.realsense_isstop):
            # Grab data from the device.                        
            frames  = self.__pipeline.wait_for_frames()  
            # Align the depth frame to color frame   
            frames = self.__align.process(frames)       
            depth_f = frames.get_depth_frame()
            color_f = frames.get_color_frame()
            if not depth_f or not color_f:
                continue            
            # Get filtered depth frame
            process_frame = self.imgprocessing(depth_f)                      
            # RGB frame and depth frame
            self.color_frame = np.asanyarray(color_f.get_data())[20:-20,20:-20]
            self.depth_frame = np.asanyarray(process_frame.get_data())[20:-20,20:-20]               
            # Show depth in color map
            self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_frame, alpha=0.03), cv2.COLORMAP_JET)
    
    def sense_obs(self):
        # Sense obstacles
        self.depth_frame[self.depth_frame <= 0] = np.inf
        self.ROI_L = self.depth_frame[self.LBox[0][0]:self.LBox[1][0], self.LBox[0][1]:self.LBox[1][1]]
        self.ROI_R = self.depth_frame[self.RBox[0][0]:self.RBox[1][0], self.RBox[0][1]:self.RBox[1][1]]
        self.ROI_M = self.depth_frame[self.MBox[0][0]:self.MBox[1][0], self.MBox[0][1]:self.MBox[1][1]]
        
        dx_L, dy_L = np.where(self.ROI_L < 1500) # [mm]
        dx_R, dy_R = np.where(self.ROI_R < 1500) 
        dx_M, dy_M = np.where(self.ROI_M < 1500)

        if dx_L.any():            
            L_i = int(np.median(dx_L)) 
            L_j = int(np.median(dy_L))          
            L_depth = self.depth_frame[]
        else:
           
        
        if dx_R.any():
            R_i = int(np.median(dx_R)) 
            R_j = int(np.median(dy_R)) 
           
        else:
          
        
        if dx_M.any():
            M_i = int(np.median(dx_M)) 
            M_j = int(np.median(dy_M))  
           
          
       
if __name__ =='__main__':      
    RS = RealSense()
    RS.realsense_info()
    RS.realsense_start()
    time.sleep(2)
    import timeit             
    while 1:
        timer = timeit.default_timer()       
        cv2.imshow('realsense',RS.realsense_get_frame())      
        print('time: {:.4f}'.format(timeit.default_timer()-timer))                
        key = cv2.waitKey(5) & 0xFF # use jetson xavier use waitKey(5)        
        if key == 27:
            RS.realsense_stop()            
            break