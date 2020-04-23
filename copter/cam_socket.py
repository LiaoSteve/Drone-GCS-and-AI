import cv2
import threading, time, logging
import socket, time, pickle, io, zlib, struct
'''
Author: LiaoSteve
>> Threading cam and socket are combined together
'''
class CamSocket():
        def __init__(self, URL, origin_size=False):            
            #-- cam
            self.__URL = URL
            self.cam_Frame = []            
            self.cam_isstop = False 
            self.origin_size = origin_size   
            self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
            #-- socket
            self.client_socket = None

        def cam_start(self):	              
            threading.Thread(target=self.cam_queryframe, daemon=False, args=()).start()

        def cam_stop(self):	   
            self.cam_isstop = True
            print('Cam stopped!')   

        def cam_getframe(self):        
            return self.cam_Frame   

        def cam_connect(self):
            self.cam_vid = cv2.VideoCapture(self.__URL)
            if not self.cam_vid.isOpened():
                raise IOError("Couldn't open webcam or video")
            if not self.origin_size:
                self.cam_vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cam_vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.__cam_video_FourCC = int(self.cam_vid.get(cv2.CAP_PROP_FOURCC))
            #self.video_FourCC2    = cv2.VideoWriter_fourcc(*"mp4v")
            self.__cam_video_fps = self.cam_vid.get(cv2.CAP_PROP_FPS)
            self.__cam_video_size = (int(self.cam_vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(self.cam_vid.get(cv2.CAP_PROP_FRAME_HEIGHT))) 
                           
        def socket_connect(self):
            #-- connect to my PC server
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.settimeout(0.1)
                self.client_socket.connect(('140.121.130.133',9998))                     
                print('*********\nsocket connect OK')
            except:
                print('socket connect fail')

        def cam_queryframe(self):                                               
            print('\n>> Cam queryframe',str(self.__URL),'started !')              
            while True:    
                try:            
                    self.socket_connect()   
                    self.cam_connect()                                          
                    while not self.cam_isstop:    
                        try:         
                            _, self.cam_Frame = self.cam_vid.read() 
                            _, frame = cv2.imencode('.jpg', self.cam_Frame, self.encode_param)                       
                            data = pickle.dumps(frame, 0)
                            size = len(data)                                                                                                                       
                            self.client_socket.sendall(struct.pack(">L", size) + data)                          
                        except Exception as e:                                        
                            print(e)                               
                            self.cam_vid.release()       
                            self.client_socket.close()
                            break                                           
                except Exception as e:   
                    time.sleep(2)                
                    print(e)                  
                
if __name__ == '__main__':    
   
    socket_cam = CamSocket(0)    
    socket_cam.cam_start() 
    time.sleep(3)
    while 1:     
        try:
            frame = socket_cam.cam_Frame        
            cv2.imshow("result", frame)
        except: continue
        if cv2.waitKey(1) & 0xFF == 27:             
            cv2.destroyAllWindows()
            break  
    
    
