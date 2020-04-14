
from cam import*
from sent_images_to_socket_server import*
from gcs import*
from realsense_cam import*
from yolo import*
import logging

logging.basicConfig(level=logging.DEBUG)
class DroneSystem(GCS, Cam, CamSocket, RealSense, YOLO):
    def __init__(self, 
                 wp_path='data/X_ground.json', connection_string= 'tcp:127.0.0.1:5762', baud = 115200, wait_ready = True, timeout = 180, heartbeat_timeout = 180,
                 gcs_ip = '140.121.130.133', gcs_port = 9999,
                 URL = 0,
                 cam_ip = '140.121.130.133', cam_port = 9998
                 ):
        GCS.__init__(self, wp_path = wp_path, connection_string = connection_string, baud = baud, wait_ready = wait_ready, timeout = timeout, gcs_ip = gcs_ip, gcs_port = gcs_port)       
        Cam.__init__(self, URL = URL)  # used for yolo
        CamSocket.__init__(self, ip = cam_ip, port = cam_port)  
        RealSense.__init__(self) # used for avoiding obstacle
        YOLO.__init__(self)
        

        self.cam_start() 
        time.sleep(1) # don't change this

        self.realsense_start()                       
        time.sleep(1) # don't change this 

        self.GCS_start()         
        self.cam_socket_start()
        time.sleep(1)    

        self.yolo_thread()
    def yolo_thread(self):
        def yolo_job():
            if not self.cam_vid.isOpened():
                raise IOError("Couldn't open webcam or video")
            while 1:                    
                if not len(self.cam_Frame):
                    continue
                image = Image.fromarray(self.cam_Frame)
                image =self.detect_image(image)
                logging.debug(' Yolo result: {}'.format(self.yolo_result))
                result = np.asarray(image)    
                cv2.imshow("result", result)  
                key = cv2.waitKey(1) & 0xFF
                if key == 27 :
                    self.cam_stop()
                    self.close_session()
                    cv2.destroyAllWindow("result")
                    break 
        threading.Thread(target=yolo_job, daemon=True, args=()).start()

   
    def OSD(self):
        try:
            status_frame = self.color_frame
            cv2.putText(img=status_frame, text=str(self.mode), org=(70, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 255, 0), fontScale=1, thickness=3) 
            self.send_img_to_server(status_frame)
        except Exception as e:
            print(e)

if __name__ =='__main__':    
    X_ground = {'lat':25.149657404957285 ,'lon':121.77688926458357}
    sitl = dronekit_sitl.start_default(lat = X_ground['lat'], lon = X_ground['lon'])
    connection_string = sitl.connection_string()  
    DS = DroneSystem(connection_string=connection_string)  

    s = input('Ready to takeoff ? [y/n]')
    if s is not 'y':
        sys.exit(0)

    DS.arm_and_takeoff(2)
    DS.mark_vehicle_home()    
    DS.generate_checkpoint()    
    time.sleep(2)
    try:
        while 1:
            #cv2.imshow('cam',DS.cam_Frame)
            cv2.imshow('realsense',DS.realsense_get_frame())
            DS.goto(DS.target,3)                             
            #DS.OSD(DS.cam_Frame)      
            key = cv2.waitKey(1) & 0xFF
            if key == 27:                
                DS.realsense_stop()                
            #timer = timeit.default_timer()            
            #print('{:.7f}'.format(timeit.default_timer()-timer))            
            if DS.dist_to_target < 1.6:
                print('>> Arrive wp:',DS.num_wp+1)
                DS.next_target()
                DS.mark_vehicle_home()
                DS.generate_checkpoint()                   
            
    except KeyboardInterrupt:
        DS.vehicle.close()
        DS.cam_stop()
        DS.realsense_stop()
        sys.exit(0)
    finally:
        DS.vehicle.close()
        DS.cam_stop()
        DS.realsense_stop()
        logging.debug('finally')
        sys.exit(0)