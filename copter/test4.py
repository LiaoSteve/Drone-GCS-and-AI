from cam import*
from realsense_cam import*

my_cam = Cam(0)    
my_cam.cam_start()       
time.sleep(1)

RS = RealSense()
RS.realsense_start()
time.sleep(1)

while 1:              
    cv2.imshow('cam',my_cam.cam_Frame)      
    cv2.imshow('realsense',RS.realsense_get_frame())
    key = cv2.waitKey(1) & 0xFF        
    if key==27:            
        my_cam.cam_stop()               
        RS.realsense_stop()
        break
