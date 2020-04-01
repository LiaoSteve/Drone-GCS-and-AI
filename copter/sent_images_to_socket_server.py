"-------------------------------- SENT IMAGE TO SERVER -----------------------"
import cv2
import socket, time, pickle, io, zlib, struct

# -- TCP --
class CamSocket():
    def __init__(self, ip, port):       
        self.cam_ip = ip
        self.cam_port = port 
        self.cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cam_sock.settimeout(1)   
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]  
    def cam_socket_connect(self):
        try:
            self.cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.cam_sock.settimeout(0.01)   
            self.cam_sock.connect((self.cam_ip,self.cam_port))
            print('Success connecting to webcam server ')
        except :            
            print('>> Error, maybe you should open webcam page or webcam server')
    def sent_img_to_server(self, frame):
        if frame is not None:
            _, frame = cv2.imencode('.jpg', frame, self.encode_param)        
            data = pickle.dumps(frame, 0)
            size = len(data)      
            try:          
                self.cam_sock.sendall(struct.pack(">L", size)+  data)
            except Exception as e:
                print(e)
                print('\n********\nwebcam connection error')
                try:
                    print('Try to connect to webcam_Server...')
                    self.cam_socket_connect()       
                    print('OK')                              
                except:
                    print('Cannot connect to webcam server\n******')
    
if __name__ == '__main__':
    my_cam_socket = CamSocket('140.121.130.133', 9998)
    my_cam_socket.cam_socket_connect()
    cap = cv2.VideoCapture(0)
    time.sleep(2)
    while 1:
        _, frame = cap.read()    
        my_cam_socket.sent_img_to_server(frame)
        cv2.imshow('img', frame)
        key = cv2.waitKey(1) & 0xFF
        if key==27: # ESC           
            cv2.destroyAllWindows()
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

