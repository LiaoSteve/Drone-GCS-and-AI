import cv2
import threading
class Cam():
        def __init__(self, URL):
            self.cam_Frame = []
            self.cam_status = False
            self.cam_isstop = False            
            self.cam_vid = cv2.VideoCapture(URL)
            if not self.cam_vid.isOpened():
                raise IOError("Couldn't open webcam or video")
            self.cam_video_FourCC = int(self.cam_vid.get(cv2.CAP_PROP_FOURCC))
            #self.video_FourCC2    = cv2.VideoWriter_fourcc(*"mp4v")
            self.cam_video_fps = self.cam_vid.get(cv2.CAP_PROP_FPS)
            self.cam_video_size = (int(self.cam_vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(self.cam_vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        def cam_start(self):	    
            print('cam started!')
            threading.Thread(target=self.cam_queryframe, daemon=True, args=()).start()
        def cam_stop(self):	   
            self.cam_isstop = True
            print('cam stopped!')   
        def cam_getframe(self):        
            return self.cam_Frame        
        def cam_queryframe(self):            
            while (not self.cam_isstop):
                self.cam_status, self.cam_Frame = self.cam_vid.read()                
                if self.cam_status == False:
                    break             
            self.cam_vid.release()   
            cv2.destroyAllWindows()
if __name__ == '__main__':
    my_cam = Cam(0)    
    my_cam.cam_start()      
    while 1:
        frame = my_cam.cam_getframe()        
        if not len(frame): # wait for frame
            continue        
        cv2.imshow('cam',frame)
        key = cv2.waitKey(1) & 0xFF        
        if key==27:            
            my_cam.cam_stop()
            break
