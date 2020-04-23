from send_images_to_socket_server import*
from gcs import*
from realsense_cam import*
from yolo import*
from cam import*
import logging
# D-LINK 4G  >> https://askubuntu.com/questions/1018375/how-do-i-install-driver-for-rtl88x2bu/1067500#1067500?newreg=3850b778a5474b4f8ae3a4028a9a3d9d

class DroneSystem(GCS, Cam, RealSense, CamSocket, YOLO):
    def __init__(self, 
                 wp_path='data/X_ground.json', connection_string= 'tcp:127.0.0.1:5762', baud = 115200, wait_ready = True, timeout = 180, heartbeat_timeout = 180,
                 gcs_ip = '140.121.130.133', gcs_port = 9999,
                 URL = 0,
                 cam_ip = '140.121.130.133', cam_port = 9998, timeout = 0.01):
        
        GCS.__init__(self, wp_path = wp_path, connection_string = connection_string, baud = baud, wait_ready = wait_ready, timeout = timeout, gcs_ip = gcs_ip, gcs_port = gcs_port)       
        Cam.__init__(self, URL = 0)
        CamSocket.__init__(self, ip = cam_ip, port = cam_port, timeout = timeout)     
        RealSense.__init__(self) # used for avoiding obstacle
        YOLO.__init__(self)        

        self.cam_start() 
        time.sleep(1) # don't change this

        self.realsense_start()                       
        time.sleep(1) # don't change this 

        self.GCS_start()               
        time.sleep(1)    

        #self.yolo_thread()

    def yolo_thread(self):
        def yolo_job():
            if not self.cam_vid.isOpened():
                raise IOError("Couldn't open webcam or video")
            while 1:                                
                if not len(self.cam_Frame) or self.yolo_thread_isstop: continue                    
                image = Image.fromarray(self.cam_Frame)
                image =self.detect_image(image)
                logging.debug(' Yolo result: {}'.format(self.yolo_result))
                result = np.asarray(image)    
                cv2.imshow("Yolo result", result)                  
                if cv2.waitKey(1) & 0xFF == 27 :          
                    self.yolo_thread_isstop = True          
                    self.close_session()                
                                
        threading.Thread(target=yolo_job, daemon=True, args=()).start()
   
    def OSD(self):
        try:
            status_frame = self.color_frame
            cv2.putText(img=status_frame, text=str(self.mode), org=(70, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 255, 0), fontScale=1, thickness=3) 
            self.send_img_to_server(status_frame)
        except Exception as e:
            print(e)

if __name__ =='__main__':  
   
    logging.basicConfig(level=logging.INFO)
    X_ground = {'lat':25.149657404957285 ,'lon':121.77688926458357}       
    #connection_string = /dev/ttyACM0
    DS = DroneSystem(connection_string='tcp:140.121.130.133:5762')      
    s = input('Ready to takeoff ? [y/n]')
    if s is not 'y':
        sys.exit(0)

    DS.arm_and_takeoff(20)
    DS.mark_vehicle_home()
    DS.generate_checkpoint()    
    time.sleep(2)
    try:
        while 1:
            cv2.imshow('cam',DS.cam_Frame)
            cv2.imshow('realsense',DS.realsense_get_frame())
            DS.goto(DS.target,groundspeed=5)                             
            DS.send_img_to_server(DS.cam_Frame)     
            
            if cv2.waitKey(2) & 0xFF == 27:                
                DS.realsense_stop()         
                DS.cam_stop()    
                
            if DS.dist_to_target < 1.6:
                print('>> Arrive wp:',DS.num_wp+1)
                DS.next_target()
                DS.mark_vehicle_home()
                DS.generate_checkpoint()                   
    except Exception as e:
        print(e)
    except KeyboardInterrupt:
        pass    
    finally:
        DS.vehicle.close()
        DS.cam_stop()
        DS.realsense_stop()
        logging.debug('finally')
        sys.exit(0)