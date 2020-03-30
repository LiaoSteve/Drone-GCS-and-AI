import cv2
import threading
class CAM(object):
        def __init__(self, URL):
            self.Frame = []
            self.status = False
            self.isstop = False            
            self.vid = cv2.VideoCapture(URL)
            if not self.vid.isOpened():
                raise IOError("Couldn't open webcam or video")
            self.video_FourCC = int(self.vid.get(cv2.CAP_PROP_FOURCC))
            #self.video_FourCC2    = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_fps = self.vid.get(cv2.CAP_PROP_FPS)
            self.video_size = (int(self.vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        def start(self):	    
            print('cam started!')
            threading.Thread(target=self.queryframe, daemon=True, args=()).start()
        def stop(self):	   
            self.isstop = True
            print('cam stopped!')   
        def getframe(self):        
            return self.Frame        
        def queryframe(self):            
            while (not self.isstop):
                self.status, self.Frame = self.vid.read()                
                if self.status == False:
                    break             
            self.vid.release()   
            cv2.destroyAllWindows()
if __name__ == '__main__':
    my_cam = CAM(0)
    my_cam.start()     
    import time 
    time.sleep(2) # wait for frame 
    while 1:
        frame = my_cam.getframe()
        cv2.imshow('cam',frame)
        key = cv2.waitKey(1) & 0xFF        
        if key==27:
            my_cam.stop()
            break
