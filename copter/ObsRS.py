# Update : 2019/12/19
# Author : Tzung-Hsien Huang

import cv2
import pyrealsense2 as rs
import numpy as np
import threading, time

class SenseObstacle(object):    
    def __init__(self, coord1, coord2):
        # The ROI coordinate
        self.roi_top_left  = coord1
        self.roi_bot_right = coord2
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale  = self.depth_sensor.get_depth_scale()
        
        self.process_frame = []
        self.frames        = []
        self.distance_map  = []
        self.color_frame, self.depth_frame, self.combine_frame = [], [], []
        
        self.is_stop = False
        self.is_obs  = False
        
        self.x_value = 0
        self.y_value = 0
        self.z_value = 0
        
    def start(self):
        # Start a thread working for getting the obstacle values as fuzzy inputs.
        print("Start the RealSense cam . ")
        R = threading.Thread(target=self.operation)
        R.setDaemon(True)
        R.start()
    
    def stop(self):
        # Stop the operation.
        self.is_stop = True
        print("Stop the RealSense.")
        self.pipeline.stop()
        cv2.destroyAllWindows()
        
    def get_frame(self):
        self.combine_frame = np.hstack((self.color_frame, self.depth_frame1,self.depth_frame))
        return self.combine_frame
    
    def getObsLocation(self):
        # Get the depth information, distance.
        self.distance_map = self.depth_image * self.depth_scale * 100 # in cm
        self.distance_map[self.distance_map<=0] = np.inf
        # Setting the Region of Interest
        self.ROI = self.distance_map[self.roi_top_left[0][1]:self.roi_bot_right[0][1], self.roi_top_left[0][0]:self.roi_bot_right[0][0]]
        # Find the points of which distance are less than 500cm
        self.d_i, self.d_j = np.where(self.ROI<=150)
        if self.d_i.any():
            self.median_i = int(np.median(self.d_i)) # for row in matrix, y-axis in figure.
            self.median_j = int(np.median(self.d_j)) # for column in matrix, x-axis in figure.
            
            self.x_value = (self.roi_bot_right[0][0] + self.roi_top_left[0][0])//2 - (self.roi_top_left[0][0] + self.median_j) # Center of the vehicle - the obstacle
            self.y_value = (self.roi_bot_right[0][1] + self.roi_top_left[0][1])//2 - (self.roi_top_left[0][1] + self.median_i)
            self.z_value = self.distance_map[self.roi_top_left[0][1] + self.median_i, self.roi_top_left[0][0] + self.median_j]
        else:
            self.x_value, self.y_value, self.z_value = np.inf, np.inf, np.inf
            
        return self.x_value, self.y_value, self.z_value
    
    def filter_setup(self):
        # Spatial filter
        self.spatial         = rs.spatial_filter()
        self.spatial.set_option(rs.option.holes_fill, 2) # default: 0
        self.spatial.set_option(rs.option.filter_smooth_alpha, 1) # default: 0.5, range:[0.25,1]
        self.spatial.set_option(rs.option.filter_smooth_delta, 50) # default: 20, range:[1, 50]
        # For longer range
        self.depth2disparity = rs.disparity_transform(True)
        self.disparity2depth = rs.disparity_transform(False)
        # Hole filling filter
        self.hole_filling    = rs.hole_filling_filter(mode=2)
        # Threshold Filter
        self.threshold       = rs.threshold_filter()
        self.threshold.set_option(rs.option.max_distance, 8) # in meter
        self.threshold.set_option(rs.option.min_distance, 0.28)
    
    def imgprocessing(self, frame_in):
        #frame = self.depth2disparity.process(frame_in)
        #frame = self.spatial.process(frame)
        #frame = self.disparity2depth.process(frame)
        frame = self.hole_filling.process(frame_in)
        self.process_frame = self.threshold.process(frame)
        return self.process_frame
    
    def operation(self):
        self.filter_setup()
        while (not self.is_stop):
            # Grab data from the device.
            self.frames  = self.pipeline.wait_for_frames()
            self.depth_f = self.frames.get_depth_frame()
            self.color_f = self.frames.get_color_frame()
            if not self.depth_f or not self.color_f:
                continue
            
            # Filtering
            self.process_frame = self.imgprocessing(frame_in=self.depth_f)
            #self.process_frame = self.depth_f   # Not filtering
            
            # Show the RGB frame. As for the Depth's one, apply the colormap.
            self.color_frame = np.asanyarray(self.color_f.get_data())
            self.depth_image = np.asanyarray(self.process_frame.get_data())
            self.depth_image1 = np.asanyarray(self.depth_f.get_data())            
            
            self.depth_frame = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
            self.depth_frame1 = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image1, alpha=0.03), cv2.COLORMAP_JET)

if __name__ == "__main__":
    print('Testing for this class function,\n press E to break the loop.')
    c1 = [(170, 100)]
    c2 = [(425, 325)]
    cent_x = (c1[0][0]+c2[0][0])//2
    cent_y = (c1[0][1]+c2[0][1])//2
    
    SO = SenseObstacle(coord1=c1, coord2=c2)
    SO.start()
    time.sleep(2)
    import timeit
    while True:
        timer = timeit.default_timer()  
        frame_out = SO.get_frame()
        if frame_out.any():
            x_location, y_location, z_distance = SO.getObsLocation()
            cv2.rectangle(frame_out, c1[0], c2[0], (0, 255, 255), 2)
            if x_location != np.inf:
                #print('x_:{x}, y_:{y}, distance:{z:.2f}cm'.format(x=x_location, y=y_location, z=z_distance))
                cv2.circle(frame_out, (cent_x-x_location, cent_y-y_location), 5, (0,0,255), -1)
            else:
                pass
                #print('clear')
            result = 'Distance:{z:.2f}cm'.format(z=z_distance)
            cv2.putText(img=frame_out,text=result,org=(175,360),fontFace=cv2.FONT_HERSHEY_DUPLEX,color=(0, 255, 255),fontScale=1)
            cv2.imshow('Test Out', frame_out) 
            print('time: {:.4f}'.format(timeit.default_timer()-timer))
                    
            if cv2.waitKey(1) & 0xFF ==27:
                print('Finish')
                SO.stop()
                break
        else:
            print("waiting for frame data.")
            