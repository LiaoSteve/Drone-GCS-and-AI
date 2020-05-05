import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs     
import cv2     
import timeit       
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 
 # Start streaming
pipeline.start(config)

# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(15):
  pipeline.wait_for_frames()
  
try:
    while True:
        t = timeit.default_timer()
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
        """---------------------------- FILTER -------------------------------------"""
        depth_to_disparity = rs.disparity_transform(True)
        disparity_to_depth = rs.disparity_transform(False)
        # https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md#spatial-edge-preserving-filter
        # SPATIAL FILTER SET:        
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, 5) # default: 0
        spatial.set_option(rs.option.filter_smooth_alpha, 0.8) # default: 0.5, range:[0.25,1]
        spatial.set_option(rs.option.filter_smooth_delta, 20)  # default: 20, range:[1, 50]
        spatial.set_option(rs.option.holes_fill, 3)
        # TEMPORAL FILTER SET:
        temporal = rs.temporal_filter()
        # HOLE FILL SET:
        hole_filling = rs.hole_filling_filter(mode=2)
        
        # FILTER START:
        filtered_depth = depth_to_disparity.process(depth_frame)
        filtered_depth = spatial.process(filtered_depth)  
        filtered_depth = temporal.process(filtered_depth)
        filtered_depth = disparity_to_depth.process(filtered_depth)
        filtered_depth = hole_filling.process(filtered_depth)
        """--------------------------  AS ARRAY  ---------------------------------"""
        #Convert images to numpy arrays        
        filtered_depth = np.asanyarray(filtered_depth.get_data())[20:-20,20:-20]        
        color_image = np.asanyarray(color_frame.get_data())[20:-20,20:-20]
        
        """--------------------------  COLOR MAP ---------------------------------"""
        filter_depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(filtered_depth, alpha=0.03), cv2.COLORMAP_BONE)
        color_image = cv2.addWeighted(color_image, 0, filter_depth_colormap, 1, 0) # shape: (440,600,3)
        
        
        """--------------------------  Show Result ---------------------------------"""
        h, w, _ = color_image.shape
        eq_w = int(w/5)
        center_L = [int(w/8) * 2, int(h/2)]
        center_M = [int(w/8) * 4, int(h/2)]
        center_R = [int(w/8) * 6, int(h/2)]
        shift = 120

        ROI_L = filtered_depth[(center_L[1]-shift):(center_L[1]+shift),(center_L[0]-eq_w):(center_L[0]+eq_w)]
        ROI_R = filtered_depth[(center_R[1]-shift):(center_R[1]+shift),(center_R[0]-eq_w):(center_R[0]+eq_w)]
        ROI_M = filtered_depth[(center_M[1]-shift):(center_M[1]+shift),(center_M[0]-eq_w):(center_M[0]+eq_w)]
        
        dx_L, dy_L = np.where(ROI_L<1500) # [mm]
        dx_R, dy_R = np.where(ROI_R<1500)
        dx_M, dy_M = np.where(ROI_M<1500)

        if dx_L.any():            
            L_i = int(np.median(dx_L)) 
            L_j = int(np.median(dy_L))    
            cv2.putText(img=color_image, text=str(ROI_L[L_i,L_j]/10)+'cm', org=((center_L[0]-50), (center_L[1]+200)), fontFace=cv2.QT_FONT_BLACK, color=(0, 255, 255), fontScale=1, thickness=1)     
            cv2.circle(color_image, (L_j+(center_L[0]-eq_w),L_i+(center_L[1]-shift)), 5, (0, 255, 255), -1)
        else:
            cv2.putText(img=color_image, text='safe', org=((center_L[0]-50), (center_L[1]+200)), fontFace=cv2.FONT_HERSHEY_COMPLEX, color=(0, 255, 255), fontScale=1, thickness=1)     
        
        if dx_R.any():
            R_i = int(np.median(dx_R)) 
            R_j = int(np.median(dy_R)) 
            cv2.putText(img=color_image, text=str(ROI_R[R_i,R_j]/10)+'cm', org=((center_R[0]-50), (center_L[1]+200)), fontFace=cv2.QT_FONT_BLACK, color=(255, 0, 255), fontScale=1, thickness=1)     
            cv2.circle(color_image, (R_j+(center_R[0]-eq_w),R_i+(center_R[1]-shift)), 5, (255, 0, 255), -1)
        else:
            cv2.putText(img=color_image, text='safe', org=((center_R[0]-50), (center_R[1]+200)), fontFace=cv2.FONT_HERSHEY_COMPLEX, color=(255, 0, 255), fontScale=1, thickness=1)    
        
        if dx_M.any():
            M_i = int(np.median(dx_M)) 
            M_j = int(np.median(dy_M))  
            cv2.putText(img=color_image, text=str(ROI_M[M_i,M_j]/10)+'cm', org=((center_M[0]-50), (center_L[1]+200)), fontFace=cv2.QT_FONT_BLACK, color=(255, 255, 0), fontScale=1, thickness=1)                 
            cv2.circle(color_image, (M_j+(center_M[0]-eq_w),M_i+(center_M[1]-shift)), 5, (255, 255, 0), -1)
        else:
            cv2.putText(img=color_image, text='safe', org=((center_M[0]-50), (center_M[1]+200)), fontFace=cv2.FONT_HERSHEY_COMPLEX, color=(255, 255, 0), fontScale=1, thickness=1)            

        cv2.rectangle(color_image, ((center_L[0]-eq_w), (center_L[1]-shift)), ((center_L[0]+eq_w), (center_L[1]+shift)), (0, 255, 255), 2)  
        cv2.rectangle(color_image, ((center_R[0]-eq_w), (center_R[1]-shift)), ((center_R[0]+eq_w), (center_R[1]+shift)), (255, 0, 255), 2)
        cv2.rectangle(color_image, ((center_M[0]-eq_w), (center_M[1]-shift)), ((center_M[0]+eq_w), (center_M[1]+shift)), (255, 255, 0), 2)   
      
        cv2.imshow('depth_colormap', color_image)       
        
        if cv2.waitKey(1) & 0xFF == 27:
            break   
        print(f'time: {timeit.default_timer()-t} sec')
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()