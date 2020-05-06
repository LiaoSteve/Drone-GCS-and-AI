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
    def __init__(self, cut_x = 20, cut_y = 20):
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
        self.cut_x = cut_x
        self.cut_y = cut_y
        self.h, self.w = (480 - 2 * self.cut_y, 640 - 2 * self.cut_x)                     
       
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
        '''frame = self.__depth2disparity.process(frame_in)
        frame = self.__spatial.process(frame)
        frame = self.__disparity2depth.process(frame)'''
        frame = self.__hole_filling.process(frame_in)
        frame = self.__threshold.process(frame)
        return frame

    def operation(self):        
        self.__realsense_log.info(" >> RealSense cam started.")
        self.filter_setup()        
        while (not self.realsense_isstop):
            # Grab data from the device.                        
            frames  = self.__pipeline.wait_for_frames()  
            # Align the depth frame to color frame   
            #frames = self.__align.process(frames)       
            depth_f = frames.get_depth_frame()
            color_f = frames.get_color_frame()
            if not depth_f or not color_f:
                continue            
            # Get filtered depth frame
            process_frame = self.imgprocessing(depth_f)                      
            # RGB frame and depth frame
            self.color_frame = np.asanyarray(color_f.get_data())[self.cut_y:-self.cut_y, self.cut_x:-self.cut_x]
            self.depth_frame = np.asanyarray(process_frame.get_data())[self.cut_y:-self.cut_y, self.cut_x:-self.cut_x]               
            # Show depth in color map
            self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_frame, alpha=0.03), cv2.COLORMAP_JET)
    
    def sense_obstacle(self, roi_points = [(100, 170), (325, 425)], thresh = 1500):
        """
        --------------------------------------------------------------------------------------------------------------------
        @ roi_points: a list like [(1,2),(3,4)] : Rectangle top-left point (1,2) and bottom-right point (3,4)
        @ thresh: Sense obstacle under thresh-distance [mm] in ROI  
        @ return: one sensed obstacle data [y, x, depth] in frame, y is row , x is colunm and depth is obstacle distance in [mm]
        --------------------------------------------------------------------------------------------------------------------
        """        
        #self.depth_frame[self.depth_frame <= 0] = np.inf
        ROI = self.depth_frame[roi_points[0][0]:roi_points[1][0], roi_points[0][1]:roi_points[1][1]]               
        dy_M, dx_M = np.where(ROI < thresh)        
        if dx_M.any():
            i = int(np.median(dy_M))
            j = int(np.median(dx_M))               
            depth = ROI[i, j]
            if depth == 0:
                return False, [None, None, None]
            y = roi_points[0][0] + i
            x = roi_points[0][1] + j
            return True, [y, x, depth]
        else:
            return False, [None, None, None]
       
if __name__ =='__main__':          
    RS = RealSense()
    RS.realsense_info()
    RS.realsense_start()
    time.sleep(2)
    import timeit    
            
    while 1:
        t = time.time()      
        img = RS.realsense_get_frame()
        ret, obs = RS.sense_obstacle(thresh=2000)           
        cv2.rectangle(img, (170, 100), (425, 325), (0, 255, 255), 2)                     
        if ret:  
            print(obs)         
            cv2.circle(img, (obs[1], obs[0]), 3, (0,0,255), -1)
            cv2.putText(img=img, text=str(obs[2])+' mm', org=(obs[1], obs[0]),fontFace=cv2.FONT_HERSHEY_DUPLEX,color=(0, 255, 0),fontScale=0.7)
                    
        cv2.imshow('realsense', img)      
        print(f'time:{time.time()-t}')         
        key = cv2.waitKey(1) & 0xFF    
        if key == 27:
            RS.realsense_stop()            
            break
    