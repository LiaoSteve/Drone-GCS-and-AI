
from cam import *
from sent_images_to_socket_server import *
from gcs import *

class DroneSystem(GCS, Cam, CamSocket):
    def __init__(self, 
                 connection_string= 'tcp:127.0.0.1:5762', baud = 115200, wait_ready = True, timeout = 180, heartbeat_timeout = 180,
                 gcs_ip = '140.121.130.133', gcs_port = 9999,
                 URL = 0,
                 cam_ip = '140.121.130.133', cam_port = 9998):
        GCS.__init__(self, connection_string = connection_string, baud = baud, wait_ready = wait_ready, timeout = timeout, ip = gcs_ip, port = gcs_port)       
        Cam.__init__(self, URL = URL)  
        CamSocket.__init__(self, ip = cam_ip, port = cam_port)      
        
        
        
        


if __name__ =='__main__':    
    
    
    target = LocationGlobalRelative(lat=wp[i][1]+1,lon=wp[i][0],alt=10)    
    while 1:
        
        goto(target, groundspeed=10)
        frame = drone.cam_getframe()          
        if not len(frame): # wait for frame
            continue   
        
        drone.sent_img_to_server(frame)
        
        cv2.imshow('',frame)        
        key = cv2.waitKey(1) & 0xFF
        if key==27:
            drone.cam_stop()
            drone.close_vehicle()
            break
    
