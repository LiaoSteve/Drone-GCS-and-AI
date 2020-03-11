import airsim
import numpy as np
import math
from math import cos,sin,radians,degrees
import os
import pprint
import cv2, time, timeit, sys
import random
import my_fuzzyController

"""--------------------------- GET DRONE INFO ----------------------------------"""
def get_position():
    PosNow = client.getMultirotorState().kinematics_estimated.position
    return list((round(PosNow.x_val,3), round(PosNow.y_val,3), round(PosNow.z_val,3)))

def get_velocity():
    v = client.getMultirotorState().kinematics_estimated.linear_velocity
    return list((round(v.x_val,2),round(v.y_val,2),round(v.z_val,2)))

def get_attitude():
    pitch, roll, yaw  = airsim.to_eularian_angles(client.simGetVehiclePose().orientation)
    return list((degrees(pitch),degrees(roll),degrees(yaw)))

"""----------------------------- CONTROL STEP ----------------------------"""
def forward():                
    attitude = get_attitude()          
    V = frd2ned_in_velocity(theta=-attitude[2], v_front=speed, v_right=0) 
    client.moveByVelocityZAsync(V[0],V[1],alt,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, attitude[2]))#.join()
    time.sleep(delay)

def backword():              
    attitude = get_attitude()                  
    V = frd2ned_in_velocity(theta=-attitude[2], v_front=-speed, v_right=0) 
    client.moveByVelocityZAsync(V[0],V[1],alt,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, attitude[2])).join()
    time.sleep(delay)

def left(v):                     
    attitude = get_attitude()            
    V = frd2ned_in_velocity(theta=-attitude[2], v_front=0, v_right=-v) 
    client.moveByVelocityZAsync(V[0],V[1],alt,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, attitude[2])).join()
    time.sleep(delay)

def right(v):               
    attitude = get_attitude()                      
    V = frd2ned_in_velocity(theta=-attitude[2], v_front=0, v_right=v) 
    client.moveByVelocityZAsync(V[0],V[1],alt,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, attitude[2])).join()
    time.sleep(delay)

def stop():                  
    attitude = get_attitude()        
    client.moveByVelocityZAsync(0,0,alt,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, attitude[2])).join()            

def turn_left():       
    attitude = get_attitude()
    attitude[2] -= 3 + random.randint(0, 5)
    print('cur_yaw: ',attitude[2])
    client.moveByVelocityZAsync(0,0,alt,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False,attitude[2]))#.join()  

def turn_right():       
    attitude = get_attitude()
    attitude[2] += 3 + random.randint(0, 5)     
    print('cur_yaw: ',attitude[2])   
    client.moveByVelocityZAsync(0,0,alt,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, attitude[2]))#.join() 

def turn_up():    
    global alt    
    attitude = get_attitude()    
    alt = alt - 0.5
    print('turn_up')
    client.moveByVelocityZAsync(0,0,alt,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, attitude[2])).join()  

def turn_yaw(heading):
    #print('turn_yaw')
    client.moveByVelocityZAsync(0,0,alt,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, heading))#.join()  

"""----------------------------------- OTHER FUNCTION -------------------------------------------"""
def frd2ned_in_velocity(theta,v_front,v_right):       
    v_frd = np.array([v_front,v_right]).reshape(2,1)
    rotation_matrix = np.array([cos(radians(theta)),sin(radians(theta)),-sin(radians(theta)),cos(radians(theta))]).reshape(2,2)
    v_ned = np.dot(rotation_matrix,v_frd).reshape(-1,)
    #print('------------------------------------------')
    #print('v_ned: ',v_ned)
    return v_ned  

# Set yaw
def yawDegree(now, goal):
    # Local frame
    delta_x = (goal[0] - now[0])
    delta_y = (goal[1] - now[1])
    theta = np.rad2deg(np.arctan2(delta_y,delta_x))
    return round(theta,1)

def SetWaypoint(All_points_name):
    wp = []
    for Obj_Name in All_points_name:
        Waypoint = client.simGetObjectPose(Obj_Name).position
        # Check 
        if not (math.isnan(Waypoint.x_val) and math.isnan(Waypoint.y_val)):
            print(">> {wp_name:} Check: OK!".format(wp_name=Obj_Name))
        else:
            print(">> {wp_name:} Nan detected, re-access.".format(wp_name=Obj_Name))
            while (math.isnan(Waypoint.x_val) or math.isnan(Waypoint.y_val)):
                Waypoint = client.simGetObjectPose(Obj_Name).position                        
        wp.append([Waypoint.x_val, Waypoint.y_val, alt])
    return wp

def set_waypoints_from_txt(path):
    try:
        wp = []
        f = open(path,'r')
        while 1:
            line = f.readline()
            if line is not '':            
                text = line.split(' ')
                wp.append( [ float(text[0]), float(text[1]), float(text[2]) ] )
                print(text)
            else:
                break
        f.close()
        return wp
    except Exception as e:
        print(e)
        sys.exit()

#---------------- GLOBAL VARIABLES --------------
alt = -4
cur_yaw = 0 # heading :+x axis
duration = 0.01
speed = 2
delay = 0.1

# the index of waypoints list
wp_i = 0
h, w = 480, 640
eq_w = int(w/6) 
center_L = [ int(w/8) * 2, int(h/2) ] # [column, row]
center_M = [ int(w/8) * 4, int(h/2) ] 
center_R = [ int(w/8) * 6, int(h/2) ]

shift = 80#120 # (up + down) = (shift/2, shift/2)

# obstacle position in middle, right, left ROI
L_i, L_j = None, None
R_i, R_j = None, None
M_i, M_j = None, None

# Fuzzy system
fz = my_fuzzyController.FuzzyControl()
fz.pre_fzprocess()

#----------------- QUADROTOR CLIENT --------------
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print(client.getMultirotorState().kinematics_estimated.orientation)

""" ==================  Read the waypoints from waypoints.txt =================="""
# Set waypoint
#wp = set_waypoints_from_txt('waypoints/waypoints.txt')

""" ==================  Read the waypoints from airsim object Wp_* =================="""

Waypoints_name = client.simListSceneObjects("Wp_.*")
print(">> Waypoint list: {ww:}".format(ww=Waypoints_name))
wp = SetWaypoint(Waypoints_name)

# Set home 
home = client.getMultirotorState().kinematics_estimated.position
print('>> Home_(x, y, alt) -> ({xx}, {yy}, {zz})'.format(xx=home.x_val, yy=home.y_val, zz=home.z_val))

# Add home into wp-list for RTL.
print (">> Add Home into wp-list for RTL ...")
wp.append([home.x_val, home.y_val, alt])
print(">> {n:} Waypoints: [x, y, alt]".format(n=len(wp)))
for i in range(len(wp)):
    print('\t[{i:}]: {wp}'.format(i=i, wp=wp[i]))


print('take off')
client.takeoffAsync()
print('take off to alt=',alt)
client.moveToZAsync(alt, velocity=1).join()
att = get_attitude()
print('pitch: {}, roll: {}, yaw: {}'.format(att[0],att[1],att[2]))

GPS = get_position()
Heading = yawDegree(GPS, wp[wp_i]) 
turn_yaw(Heading)

try:
    while wp_i < len(wp):
        #t = timeit.default_timer()
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPlanner, pixels_as_float=True, compress=False),
                                                airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])
        if not responses:
            client.moveByVelocityAsync(vx=0, vy=0, vz=0, duration=1).join()
            print("** Wait for data. **")
            continue
        try:
            # For depth
            response = responses[0]
            img1d    = np.array(response.image_data_float, dtype=np.float)
            temp     = img1d
            temp2    = np.reshape(temp, (responses[0].height, responses[0].width))

            # Depth data transformating
            img1d = img1d * 4 + 20
            img1d[img1d>255] = 255
            img2d = np.reshape(img1d, (responses[0].height, responses[0].width))
            depth = np.array(img2d,dtype=np.uint8)             

            color    = responses[1]
            imgcolor = np.fromstring(color.image_data_uint8, dtype=np.uint8)
            imgcolor = imgcolor.reshape(responses[1].height, responses[1].width, -1)   
            if imgcolor.shape[2] == 4:            
                imgcolor = cv2.cvtColor(imgcolor,cv2.COLOR_RGBA2BGR)
            depth  = cv2.cvtColor(depth,cv2.COLOR_GRAY2BGR)
            
            depth = cv2.addWeighted(imgcolor, 0.75, depth, 0.25, 0)
        except Exception as e:
            #print(e)
            pass            
        
        GPS = get_position()
        V_global = get_velocity()                                 
        Heading = yawDegree(GPS, wp[wp_i])       

        """========================  AVOID DECITION  ================================="""              
        ROI_M = temp2[(center_M[1]-shift):(center_M[1]+shift),(center_M[0]-eq_w):(center_M[0]+eq_w)] *100 # [cm]  

        dx_M, dy_M = np.where(ROI_M < 1000) #  cm                
        
        if dx_M.any():
            M_i = int(np.median(dx_M)) 
            M_j = int(np.median(dy_M))             

            vx, vy, vz, turn_yaw, cost_t = fz.fzprocess(delta_x=M_i, delta_y=M_j, distance=ROI_M[M_i,M_j])
            attitude = get_attitude()            
            V = frd2ned_in_velocity(theta=-attitude[2]+turn_yaw, v_front=vx, v_right=vy) 
            client.moveByVelocityAsync(vx=V[0], vy=V[1], vz=vz, duration=1)
            cv2.putText(img=depth, text='{:.2f}m'.format(ROI_M[M_i,M_j]), org=((center_M[0]-50), (center_L[1]+200)), fontFace=cv2.QT_FONT_BLACK, color=(255, 255, 0), fontScale=1, thickness=1)                 
            cv2.circle(depth, (M_j+(center_M[0]-eq_w),M_i+(center_M[1]-shift)), 4, (255, 255, 0), -1)                 
        else:   
            Heading = yawDegree(GPS, wp[wp_i])                          
            cv2.putText(img=depth, text='safe', org=((center_M[0]-50), (center_M[1]+200)), fontFace=cv2.FONT_HERSHEY_COMPLEX, color=(255, 255, 0), fontScale=1, thickness=1) 
            client.moveToPositionAsync(wp[wp_i][0], wp[wp_i][1], alt, velocity=speed, yaw_mode=airsim.YawMode(False, Heading))                       

        dist_to_waypoint = round((round((GPS[0] - wp[wp_i][0]),3)**2 + round((GPS[1] - wp[wp_i][1]),3)**2),3)**0.5
        # Check if reach the waypoint(x,y)
        if dist_to_waypoint <= 1.5:             
            stop()
            print(">> Arrived at wp{Nwp:}({x:}, {y:}, {z:})!".format(Nwp=wp_i+1, x=GPS[0], y=GPS[1], z=GPS[2]))
            if wp_i >= len(wp) -1:
                print('\n ==== Mission Complete ! ====')  
                turn_yaw(0)              
                print(">> LAND")
                client.landAsync().join()
                client.armDisarm(False)
                cv2.destroyAllWindows()
                break
            wp_i += 1            
            Heading = yawDegree(GPS, wp[wp_i]) 
            turn_yaw(Heading)        

        cv2.putText(img=depth, text='Pos[x,y,z]: [{:.1f}, {:.1f}, {:.1f}]'.format(GPS[0],GPS[1],GPS[2]), org=(10, 18), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 0, 0), fontScale=0.5, thickness=1) 
        cv2.putText(img=depth, text='V_global[x,y,z]: [{:.1f}, {:.1f}, {:.1f}]'.format(V_global[0],V_global[1],V_global[2]), org=(10, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 0, 0), fontScale=0.5, thickness=1) 
        cv2.putText(img=depth, text='way_point: {}/{}, dist2waypoint: {:.2f} m'.format(wp_i+1, len(wp), dist_to_waypoint), org=(10, 60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 0, 0), fontScale=0.5, thickness=1) 
        cv2.rectangle(depth, ((center_M[0]-eq_w), (center_M[1]-shift)), ((center_M[0]+eq_w), (center_M[1]+shift)), (255, 255, 0), 2)  
        
        cv2.imshow('depth',depth)
        #cv2.imshow("color", imgcolor)
        key = cv2.waitKey(1) & 0xFF

        if key == 27 or key == ord('q'):
            cv2.destroyAllWindows()
            break        
        
        #print('time:{:.2f} sec'.format(timeit.default_timer()-t))   

except Exception as e:
    print(e)
    client.reset()
finally:
    cv2.destroyAllWindows()
    client.reset()