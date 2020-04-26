from camsocket import*
from gcs import*
from realsense_cam import*
from yolo import*
from cam import*
'''
-------------------------------------------------------------------------------------------------------------------------------------------------------
@ Author        : LiaoSteve
@ D-LINK driver : https://askubuntu.com/questions/1018375/how-do-i-install-driver-for-rtl88x2bu/1067500#1067500?newreg=3850b778a5474b4f8ae3a4028a9a3d9d
-------------------------------------------------------------------------------------------------------------------------------------------------------
'''
class DroneSystem(GCS, Cam, CamSocket, RealSense, YOLO):
    def __init__(self, 
                 wp_path='data/X_ground.json', connection_string= 'tcp:127.0.0.1:5762', baud = 115200, wait_ready = True, timeout = 180, heartbeat_timeout = 180,
                 gcs_ip = '140.121.130.133', gcs_port = 9999,
                 URL = 0,
                 cam_ip = '140.121.130.133', cam_port = 9998, socket_timeout = 0.01):
        # -- object detected and identified 
        YOLO.__init__(self) 
        # -- ground control station
        GCS.__init__(self, wp_path = wp_path, connection_string = connection_string, baud = baud, wait_ready = wait_ready, timeout = timeout, gcs_ip = gcs_ip, gcs_port = gcs_port)       
        # -- thread cam
        Cam.__init__(self, URL = '../../2019-11-19.mp4')
        # -- send images to my django server
        CamSocket.__init__(self, ip = cam_ip, port = cam_port, timeout = socket_timeout)     
        # -- use thread realsense depth camera to detect obstacle 
        RealSense.__init__(self) 
          
        
        # -- start the class above
        self.cam_start() 
        time.sleep(1)
        # -- don't change time.sleep(1)
        self.realsense_start()                       
        time.sleep(1) 
        # -- don't change time.sleep(1)
        self.GCS_start()    
        self.cam_socket_start()           
        time.sleep(3)   
        
        # -- drone system variable
        self.trash_num   = 0
        self.cap_num     = 0
        self.plastic_bag_num = 0

        self.ds_log = logging.getLogger(' dronesystem')
        self.ds_log.setLevel(logging.INFO)
        self.yolo_thread_isstop = False
        if not self.yolo_thread_isstop: self.yolo_thread_start()

    def mark_yolo_result(self):
        trash_num = cap_num = plastic_bag_num = 0
        for i in range(self.yolo_result.__len__()):
            if self.yolo_result[i][0] == self.class_names[0]:
                trash_num += 1
            if self.yolo_result[i][0] == self.class_names[1]:
                cap_num += 1
            if self.yolo_result[i][0] == self.class_names[2]:
                plastic_bag_num += 1                
        self.trash_num   += trash_num
        self.cap_num     += cap_num
        self.plastic_bag_num += plastic_bag_num                        
        data = {}
        data['channel'] = '00001'            
        data['latitude'] = self.cur_pos.lat
        data['longitude'] = self.cur_pos.lon
        data['timestamp'] = str(datetime.now())
        data['trash_num'] = str(trash_num)
        data['cap_num'] = str(cap_num)
        data['plastic_bag_num'] = str(plastic_bag_num)
        message = json.dumps(data)
        self.send_data_to_MapServer(message) 
        # to do : revise plastic_num on server
    def yolo_thread_start(self):
        def yolo_job():            
            while 1:        
                try:   
                    if self.yolo_thread_isstop: break                             
                    image = Image.fromarray(self.cam_Frame)
                    image = self.detect_image(image)
                    #self.ds_log.info(' Yolo result: {}'.format(self.yolo_result)) 
                    self.mark_yolo_result()                   
                    result = np.asarray(image) 
                    result = cv2.resize(result, (640,480))                      
                    self.send_img_to_server(result)
                    time.sleep(0.2)
                except:
                    pass              
            self.close_session()          
                                           
        threading.Thread(target = yolo_job, daemon = True, args=()).start()
   
    def OSD(self):
        try:
            status_frame = self.color_frame
            cv2.putText(img=status_frame, text=str(self.mode), org=(70, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 255, 0), fontScale=1, thickness=3) 
            self.send_img_to_server(status_frame)
        except Exception as e:
            self.ds_log.warning(type(e))

if __name__ =='__main__':        
    
    X_ground = {'lat':25.149657404957285 ,'lon':121.77688926458357}       
    #connection_string = /dev/ttyACM0
    DS = DroneSystem(connection_string='tcp:140.121.130.133:5762')    
    time.sleep(7)
    s = input('Ready to takeoff ? [y/n]')
    if s is not 'y':
        sys.exit(0)
    try:
        DS.arm_and_takeoff(10)
        DS.mark_vehicle_home()
        DS.generate_checkpoint()    
        time.sleep(2)
    
        while 1:                      
            DS.goto(DS.target,groundspeed = 2)                                          
            cv2.imshow('realsense',DS.realsense_get_frame())
            if DS.dist_to_target < 1.6:
                DS.ds_log.info(f'>> Arrive wp:{DS.num_wp+1}')
                DS.next_target()
                DS.mark_vehicle_home()
                DS.generate_checkpoint()                 
            if cv2.waitKey(500) & 0xFF == 27:                
               break
    except Exception as e:
        DS.ds_log(e)               
    finally:        
        print('finally')               
        DS.vehicle.close()
        DS.cam_stop()
        DS.realsense_stop()              
        sys.exit(0)