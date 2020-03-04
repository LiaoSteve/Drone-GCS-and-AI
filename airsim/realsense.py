import airsim
import numpy as np
import os
import pprint
import cv2, time, timeit, sys
from manual_mode_demo import *
from pynput.keyboard import Key, Controller

def get_position():
    PosNow = client.getMultirotorState().kinematics_estimated.position
    return list((round(PosNow.x_val,2), round(PosNow.y_val,2), round(PosNow.z_val,2)))

def get_velocity():
    v = client.getMultirotorState().kinematics_estimated.linear_velocity
    return list((round(v.x_val,2),round(v.y_val,2),round(v.z_val,2)))

def get_attitude():
    pitch, roll, yaw  = airsim.to_eularian_angles(client.simGetVehiclePose().orientation)
    return list((degrees(pitch),degrees(roll),degrees(yaw)))
    
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print('take off')
client.takeoffAsync().join()

keyboard = Controller()

h, w= 480, 640
eq_w = int(w/5)
center_L = [int(w/8) * 2, int(h/2)]
center_M = [int(w/8) * 4, int(h/2)]
center_R = [int(w/8) * 6, int(h/2)]
shift = 120
# obstacle position in middle, right, left ROI
L_i, L_j = None, None
R_i, R_j = None, None
M_i, M_j = None, None
def realsense():    
    global cur_yaw
    print('**********\nRealsense start')     
    try:
        while 1:
            t = timeit.default_timer()
            responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPlanner, pixels_as_float=True, compress=False),
                                                airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])
            if not responses:
                client.moveByVelocityAsync(vx=0, vy=0, vz=0, duration=1).join()
                print("** Wait for data. **")
                continue

            # For depth
            response = responses[0]
            img1d    = np.array(response.image_data_float, dtype=np.float)
            temp     = img1d
            temp2    = np.reshape(temp, (responses[0].height, responses[0].width))

            # Depth data transformating
            img1d = img1d * 5 + 20
            img1d[img1d>255] = 255
            img2d = np.reshape(img1d, (responses[0].height, responses[0].width))
            depth = np.array(img2d,dtype=np.uint8)             

            color    = responses[1]
            imgcolor = np.fromstring(color.image_data_uint8, dtype=np.uint8)
            imgcolor = imgcolor.reshape(responses[1].height, responses[1].width, -1)    
            depth  = cv2.cvtColor(depth,cv2.COLOR_GRAY2RGB)
            
            depth = cv2.addWeighted(imgcolor, 0.6, depth, 0.4, 0)

            ROI_L = temp2[(center_L[1]-shift):(center_L[1]+shift),(center_L[0]-eq_w):(center_L[0]+eq_w)]
            ROI_R = temp2[(center_R[1]-shift):(center_R[1]+shift),(center_R[0]-eq_w):(center_R[0]+eq_w)]
            ROI_M = temp2[(center_M[1]-shift):(center_M[1]+shift),(center_M[0]-eq_w):(center_M[0]+eq_w)]

            dx_L, dy_L = np.where(ROI_L<10) 
            dx_R, dy_R = np.where(ROI_R<10)
            dx_M, dy_M = np.where(ROI_M<10) #  in meter                        

            if dx_L.any():            
                L_i = int(np.median(dx_L)) 
                L_j = int(np.median(dy_L))    
                cv2.putText(img=depth, text='{:.2f}m'.format(ROI_L[L_i,L_j]), org=((center_L[0]-50), (center_L[1]+200)), fontFace=cv2.QT_FONT_BLACK, color=(0, 255, 255), fontScale=1, thickness=1)     
                cv2.circle(depth, (L_j+(center_L[0]-eq_w),L_i+(center_L[1]-shift)), 4, (0, 255, 255), -1)
            else:
                cv2.putText(img=depth, text='safe', org=((center_L[0]-50), (center_L[1]+200)), fontFace=cv2.FONT_HERSHEY_COMPLEX, color=(0, 255, 255), fontScale=1, thickness=1)     
            
            if dx_R.any():
                R_i = int(np.median(dx_R)) 
                R_j = int(np.median(dy_R)) 
                cv2.putText(img=depth, text='{:.2f}m'.format(ROI_R[R_i,R_j]), org=((center_R[0]-50), (center_L[1]+200)), fontFace=cv2.QT_FONT_BLACK, color=(255, 0, 255), fontScale=1, thickness=1)     
                cv2.circle(depth, (R_j+(center_R[0]-eq_w),R_i+(center_R[1]-shift)), 4, (255, 0, 255), -1)
            else:
                cv2.putText(img=depth, text='safe', org=((center_R[0]-50), (center_R[1]+200)), fontFace=cv2.FONT_HERSHEY_COMPLEX, color=(255, 0, 255), fontScale=1, thickness=1)    
            
            if dx_M.any():
                M_i = int(np.median(dx_M)) 
                M_j = int(np.median(dy_M))                 
                cv2.putText(img=depth, text='{:.2f}m'.format(ROI_M[M_i,M_j]), org=((center_M[0]-50), (center_L[1]+200)), fontFace=cv2.QT_FONT_BLACK, color=(255, 255, 0), fontScale=1, thickness=1)                 
                cv2.circle(depth, (M_j+(center_M[0]-eq_w),M_i+(center_M[1]-shift)), 4, (255, 255, 0), -1)                 
            else:
                cv2.putText(img=depth, text='safe', org=((center_M[0]-50), (center_M[1]+200)), fontFace=cv2.FONT_HERSHEY_COMPLEX, color=(255, 255, 0), fontScale=1, thickness=1) 

            GPS = get_position()
            V_global = get_velocity() 

            cv2.putText(img=depth, text='Pos[x,y,z]: [{:.1f}, {:.1f}, {:.1f}]'.format(GPS[0],GPS[1],GPS[2]), org=(10, 18), fontFace=cv2.FONT_HERSHEY_DUPLEX, color=(0, 0, 0), fontScale=0.5, thickness=1) 
            cv2.putText(img=depth, text='V_global[x,y,z]: [{:.1f}, {:.1f}, {:.1f}]'.format(V_global[0],V_global[1],V_global[2]), org=(10, 40), fontFace=cv2.FONT_HERSHEY_DUPLEX, color=(0, 0, 0), fontScale=0.5, thickness=1) 
                        
            cv2.rectangle(depth, ((center_L[0]-eq_w), (center_L[1]-shift)), ((center_L[0]+eq_w), (center_L[1]+shift)), (0, 255, 255), 2)  
            cv2.rectangle(depth, ((center_R[0]-eq_w), (center_R[1]-shift)), ((center_R[0]+eq_w), (center_R[1]+shift)), (255, 0, 255), 2)
            cv2.rectangle(depth, ((center_M[0]-eq_w), (center_M[1]-shift)), ((center_M[0]+eq_w), (center_M[1]+shift)), (255, 255, 0), 2)    
                                 
            cv2.rectangle(imgcolor, (255,170), (385,300), (0,255,255), 2)

            cv2.imshow('depth',depth)
            #cv2.imshow("color", imgcolor)
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord('q'):
                cv2.destroyAllWindows()
                break
            
            #print('time:{:.2f} sec'.format(timeit.default_timer()-t))
    except Exception as e:
        print(e)
    finally:
        print('client.reset()')
        pass
        #client.reset()
        #client.enableApiControl(False) 

t = threading.Thread(target=realsense, daemon=True)
t.start()

lidarTest = LidarTest()
drone_manual_control = tk.Tk()  
drone_manual_control.bind_all('<Key>', lidarTest.key)
drone_manual_control.mainloop() 
