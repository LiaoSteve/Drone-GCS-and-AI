"-------------------------------- SENT IMAGE TO SERVER -----------------------"
import cv2
import socket, time, pickle, io, zlib, struct
class CAM_SOCKET(object):
    def __init__(self, ip, port):       
        self.ip = ip
        self.port = port 
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(1)   
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]  
    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(0.01)   
            self.sock.connect((self.ip,self.port))
            print('Success connecting to webcam server ')
        except :            
            print('>> Error to connect to webcam_server\nmaybe  should open webcam page or webcam server')
    def sent_img_to_server(self, frame):
        if frame is not None:
            _, frame = cv2.imencode('.jpg', frame, self.encode_param)        
            data = pickle.dumps(frame, 0)
            size = len(data)      
            try:          
                self.sock.sendall(struct.pack(">L", size)+  data)
            except Exception as e:
                print(e)
                print('\n********\nwebcam connection error')
                try:
                    print('try to connect to webcam_Server\n')
                    self.connect()       
                    print('success')                              
                except:
                    print('cannot connect to webcam server\n******')
    
if __name__ == '__main__':
    my_cam_socket = CAM_SOCKET('140.121.130.133', 9998)
    my_cam_socket.connect()
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