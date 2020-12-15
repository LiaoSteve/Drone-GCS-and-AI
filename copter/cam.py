import cv2
import threading, time
import logging
'''
-----------------------------------
@ Author : LiaoSteve            
-----------------------------------'''
class Cam():
        def __init__(self, URL, original_size = False):
            self.__cam_log = logging.getLogger(__name__)            
            self.__cam_log.setLevel(logging.INFO)                      
            
            self.__URL = URL
            self.cam_Frame = []  
            self.cam_state = None         
            self.cam_isstop = False     
            self.__original_size = original_size    
            self.cam_connect()        
            
        def cam_connect(self):
            self.cam_vid = cv2.VideoCapture(self.__URL)          
            if not self.cam_vid.isOpened():      
                return 0          
            if not self.__original_size:
                self.cam_vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cam_vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
            self.__cam_video_FourCC = int(self.cam_vid.get(cv2.CAP_PROP_FOURCC))
            #self.__cam_video_FourCC2    = cv2.VideoWriter_fourcc(*"mp4v")
            self.__cam_video_fps = self.cam_vid.get(cv2.CAP_PROP_FPS)
            self.__cam_video_size = (int(self.cam_vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(self.cam_vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    
        def cam_start(self):	              
            threading.Thread(target=self.cam_queryframe, daemon=True, args=()).start()

        def cam_stop(self):	   
            self.cam_isstop = True               
            self.cam_vid.release()  
            self.__cam_log.info(' >> Stop the cam .')

        def cam_getframe(self):        
            return self.cam_Frame     

        def cam_queryframe(self):                       
            self.__cam_log.info(' >> Cam {} start.'.format(self.__URL))        
            while (not self.cam_isstop):                
                self.cam_state, self.cam_Frame = self.cam_vid.read()                 
                if not self.cam_state:   
                    self.cam_vid.release() 
                    time.sleep(1)                                
                    self.cam_connect()   
                              
             
            

if __name__ == '__main__':    
    my_cam = Cam(0)    
    my_cam.cam_start()       
    time.sleep(1)   
    while 1:                
        try:cv2.imshow('cam',my_cam.cam_Frame)                
        except:continue        
        if cv2.waitKey(5) & 0xFF ==27:            
            my_cam.cam_stop()
            break
    
