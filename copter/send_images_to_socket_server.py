"-------------------------------- SENT IMAGE TO SERVER -----------------------"
import cv2
import socket, time, pickle, io, zlib, struct
import logging

# -- TCP 
class CamSocket():
    def __init__(self, ip, port, timeout):    
        # -- logger
        self.__camsocket_log = logging.getLogger('gcs')            
        self.__camsocket_log.setLevel(logging.WARNING)  

        self.__cam_ip = ip
        self.__cam_port = port         
        self.__timeout = timeout        
        self.__encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50] 

    def cam_socket_start(self):
        try:
            self.__cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__cam_sock.settimeout(self.__timeout)   
            self.__cam_sock.connect((self.__cam_ip,self.__cam_port))
            self.__camsocket_log.warning('Cam socket OK ')
        except :                       
            self.__camsocket_log.warning('Cam socket error, maybe you should open webcam page or webcam server')
    
    def send_img_to_server(self, frame):
        if frame is not None:
            _, frame = cv2.imencode('.jpg', frame, self.__encode_param)        
            data = pickle.dumps(frame, 0)
            size = len(data)                
            try:          
                self.__cam_sock.sendall(struct.pack(">L", size)+  data)
            except Exception as e:                               
                self.__cam_sock.close()                                 
                self.cam_socket_start()                                                    
                self.__camsocket_log.warning(type(e))
    
if __name__ == '__main__':
    from cam import*
    #from yolo import*    

    #my_yolo = YOLO()
    my_cam_socket = CamSocket('140.121.130.133', 9998, timeout = 0.01)
    my_cam_socket.cam_socket_start()    
    cam = Cam(0)
    cam.cam_start()
    time.sleep(2)
    while 1:      
        #image = Image.fromarray(cam.cam_Frame)
        #image = my_yolo.detect_image(image)
        #result = np.asarray(image)
        #my_cam_socket.send_img_to_server(result)   
        my_cam_socket.send_img_to_server(cam.cam_Frame)      
        cv2.imshow('img', cam.cam_Frame)
        if cv2.waitKey(100) & 0xFF == 27: # ESC                    
            break

        
''' # -- UDP --
class CamSocket():
    def __init__(self, ip, port):       
        self.cam_ip = ip
        self.cam_port = port 
        self.cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)                
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]          
        print('Cam_socket start ...')               
    def sent_img_to_server(self, frame):        
        if frame is not None:
            _, frame = cv2.imencode('.jpg', frame, self.encode_param)        
            data = pickle.dumps(frame, 0)
            size = len(data)      
            try:          
                self.cam_sock.sendto(struct.pack(">L", size)+  data, (self.cam_ip,self.cam_port))
            except Exception as e:
                print(e)
    
if __name__ == '__main__':
    my_cam_socket = CamSocket('140.121.130.133', 9998)    
    cap = cv2.VideoCapture(0)    
    while 1:
        _, frame = cap.read() 
        if not len(frame):
            continue   
        my_cam_socket.sent_img_to_server(frame)
        cv2.imshow('img', frame)
        key = cv2.waitKey(1) & 0xFF
        if key==27: # ESC           
            cv2.destroyAllWindows()
            break '''

