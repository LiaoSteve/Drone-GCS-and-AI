# Topic  : Apply Fuzzy control system on Aircraft in AirSim.
# Author : Tzung-Hsien Huang 
# Update : 2019/01/08
# >> Add "SEND" to prevent sending repeat cmd to the drone.
# >> Add Threading to move forward with current alt, and move to the wp's alt after 5sec.


import airsim
import numpy as np
import os
import pprint
import cv2, time, sys, threading, math
import FuzzyControlA, FuzzyControlB
from math import degrees

def adjH():
    global tag_time           
    while True:     
        tag_time += 1
        time.sleep(1)        


def LocalFrameV(vx, vy, ahead):
    New_vx = vx * np.cos(np.deg2rad(ahead)) - vy * np.sin(np.deg2rad(ahead))
    New_vy = vy * np.cos(np.deg2rad(ahead)) + vx * np.sin(np.deg2rad(ahead))
    return New_vx, New_vy

# Define the process in ROI
def ROI_process(distance_map):
    # ROI setup
    ROI = distance_map[170:300,255:385] * 100
    d_i, d_j = np.where(ROI<=1000)    
    if d_i.any():
        median_i = int(np.median(d_i))
        median_j = int(np.median(d_j))
        
        x_value = (255 + 385)//2 - (255 + median_j) # Center of the vehicle - the obstacle
        y_value = (170 + 300)//2 - (170 + median_i)
        z_value = ROI[median_i, median_j]
        return median_i, median_j, x_value, y_value, z_value
    else:
        x_value, y_value, z_value = np.inf, np.inf, np.inf
        return False

# Get position
def GetPos():
    PosNow = client.getMultirotorState().kinematics_estimated.position
    return [round(PosNow.x_val,4), round(PosNow.y_val,4), round(PosNow.z_val,4)] 

def get_velocity():
    v = client.getMultirotorState().kinematics_estimated.linear_velocity
    return list((round(v.x_val,2),round(v.y_val,2),round(v.z_val,2)))

def get_attitude():
    pitch, roll, yaw  = airsim.to_eularian_angles(client.simGetVehiclePose().orientation)
    return list((degrees(pitch),degrees(roll),degrees(yaw)))
# Set yaw
def yawDegree(now, goal):
    # Local frame
    delta_x = (goal[0] - now[0])
    delta_y = (goal[1] - now[1])
    theta = np.rad2deg(np.arctan2(delta_y,delta_x))
    return round(theta,4)

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

# Parameters Initialize
alt      = -3   # Define the flight alt.
wp_i     = 0
velocity = 2    # m/s
tag_time = 0
th2 = threading.Thread(target=adjH)
th2.setDaemon(True)
path = 'record_position/A.txt' #record position
# Fuzzy system
fzA = FuzzyControlA.FuzzyControl()
fzB = FuzzyControlB.FuzzyControl()
fzA.pre_fzprocess()
fzB.pre_fzprocess()
cuttrnt_fuzzy_name = 'A'
# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("\n ==== Information ====")
# Set home 
home = client.getMultirotorState().kinematics_estimated.position
print('>> Home_(x, y, z) -> ({xx}, {yy}, {zz})'.format(xx=home.x_val, yy=home.y_val, zz=home.z_val))

# Set waypoint
Waypoints_name = client.simListSceneObjects("Wp_.*")
print(">> Waypoint list: {ww:}".format(ww=Waypoints_name))
wp = SetWaypoint(Waypoints_name)

#wp = set_waypoints_from_txt('../waypoints/africa_waypoints.txt')

# Add home into wp-list for RTL.
""" print (">> Add Home into wp-list for RTL ...")
wp.append([home.x_val, home.y_val, alt]) """
num_wp = len(wp)
print(">> {n:} Waypoints: [x, y, z]".format(n=num_wp))
for i in range(len(wp)):
    print('\t[{i:}]: {wp}'.format(i=i, wp=wp[i]))

# Takeoff
print("\n ==== Takeoff ====")
client.takeoffAsync()
client.moveToZAsync(-3, velocity=1)
while 1:
    GPS = GetPos()  
    if abs(GPS[2])> 3*0.95:        
        break    
    if os.path.exists(path):
        f= open(path,"a+")
    else:
        f= open(path,"w+")                    
    try:            
        f.write(str(GPS[0])+' '+str(GPS[1])+' '+str(GPS[2])+' \n')    
        f.close()  
        time.sleep(0.5)   
    except Exception as e:
        print('error: ',e)        
        pass
#print(">> Adjust altitude to {hight:}(m)".format(hight=abs(alt)))
#client.moveToZAsync(-3, velocity=1).join()
print(">> Arrived at alt: {hight:}(m)".format(hight=abs(alt)))
print("\n ==== Start Mission =====")
print(">> Velocity:{v:}".format(v=velocity))

SEND = True
# Main
th2.start()
while wp_i < (num_wp):   
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPlanner, pixels_as_float=True, compress=False),
                                    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])
    if not responses:
        client.moveByVelocityAsync(vx=0, vy=0, vz=0, duration=1).join()
        print("** Wait for data. **")
        continue    
    if os.path.exists(path):
        f= open(path,"a+")
    else:
        f= open(path,"w+")                    
    try:    
        GPS = GetPos()
        f.write(str(GPS[0])+' '+str(GPS[1])+' '+str(GPS[2])+' \n')    
        f.close()     
    except Exception as e:
        print('error: ',e)
        print(tag_time)
        pass

    # For depth
    try:
        response = responses[0]
        img1d    = np.array(response.image_data_float, dtype=np.float)
        temp     = img1d
        temp2    = np.reshape(temp, (responses[0].height, responses[0].width))
        # Depth data transformating
        img1d = img1d*3.5+30
        img1d[img1d>255] = 255
        img2d = np.reshape(img1d, (responses[0].height, responses[0].width))
        depth = np.array(img2d,dtype=np.uint8)        
        
        # For color
        color    = responses[1]
        imgcolor = np.fromstring(color.image_data_uint8, dtype=np.uint8)
        imgcolor = imgcolor.reshape(responses[1].height, responses[1].width, -1)    
        if imgcolor.shape[2] == 4:            
            imgcolor = cv2.cvtColor(imgcolor,cv2.COLOR_RGBA2BGR)        
        # Show information
        txt = str(temp2[240,320])
        #cv2.circle(depth, (320,240), 5, (0,0,255), -1)
        #cv2.putText(depth, txt, (325, 238), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
        cv2.rectangle(depth, (255,170), (385,300), (0,255,255), 2)
        cv2.rectangle(imgcolor, (255,170), (385,300), (0,255,255), 2)
    except:
        pass
    # Get GPS data
    GPS = GetPos()    
    V_global = get_velocity()
    dist_to_waypoint = math.sqrt(abs(GPS[0] - wp[wp_i][0])**2 + abs(GPS[1] - wp[wp_i][1])**2)  
    
    Heading = yawDegree(GPS, wp[wp_i])

    cv2.putText(img=imgcolor, text='Pos [x,y,z]: [{:.1f}, {:.1f}, {:.1f}]'.format(GPS[0],GPS[1],GPS[2]), org=(10, 18), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 0, 0), fontScale=0.4, thickness=5) 
    cv2.putText(img=imgcolor, text='V_global [x,y,z]: [{:.1f}, {:.1f}, {:.1f}]'.format(V_global[0],V_global[1],V_global[2]), org=(10, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 0, 0), fontScale=0.4, thickness=5) 
    cv2.putText(img=imgcolor, text='way_point: {}/{}, dist: {:.2f} m, Fuzzy_name: {}'.format(wp_i+1, len(wp), dist_to_waypoint, cuttrnt_fuzzy_name), org=(10, 60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 0, 0), fontScale=0.4, thickness=5)
    
    cv2.putText(img=imgcolor, text='Pos [x,y,z]: [{:.1f}, {:.1f}, {:.1f}]'.format(GPS[0],GPS[1],GPS[2]), org=(10, 18), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(255, 255, 255), fontScale=0.4, thickness=1) 
    cv2.putText(img=imgcolor, text='V_global [x,y,z]: [{:.1f}, {:.1f}, {:.1f}]'.format(V_global[0],V_global[1],V_global[2]), org=(10, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(255, 255, 255), fontScale=0.4, thickness=1) 
    cv2.putText(img=imgcolor, text='way_point: {}/{}, dist: {:.2f} m, Fuzzy_name: {}'.format(wp_i+1, len(wp), dist_to_waypoint, cuttrnt_fuzzy_name), org=(10, 60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(255, 255, 255), fontScale=0.4, thickness=1)
    
   
    # Movement decision
    if ROI_process(temp2):            
        SEND = True
        # Depth data processing
        m_i, m_j, Horizontal_value, Vertical_value, Forward_value = ROI_process(temp2)
        #print(">> Recieve data: X= {VH:}, Y= {VV:}, Dist= {VF:}".format(VH=Horizontal_value, VV=Vertical_value, VF=Forward_value))        
        # Fuzzy Control        
        if ( dist_to_waypoint <= 5):
            cuttrnt_fuzzy_name = 'B'
            V_Hor, V_Ver, V_For, cost_time = fzB.fzprocess(delta_x=Horizontal_value, delta_y=Vertical_value, distance=Forward_value)
        else:
            cuttrnt_fuzzy_name = 'A'
            V_Hor, V_Ver, V_For, cost_time = fzA.fzprocess(delta_x=Horizontal_value, delta_y=Vertical_value, distance=Forward_value)
        # Round
        V_Hor = round(V_Hor,2)
        V_Ver = round(V_Ver,2)
        V_For = round(V_For,2)
        #print(">> Defuzzy data: VH= {VH:}, VV= {VV:}, VF= {VF:}".format(VH=V_Hor, VV=V_Ver, VF=V_For))
        # Calculate new velocity
        V_new_x, V_new_y = LocalFrameV(vx=V_For, vy=V_Hor, ahead=Heading)
        client.moveByVelocityAsync(vx=V_new_x, vy=V_new_y, vz=V_Ver, duration=1)
        # Show info. in color frame        
       
        txt1 = str(round(temp2[170+m_i,255+m_j], 2))
        cv2.putText(imgcolor, txt1, (257+m_j, 168+m_i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 30,255), 2)        
        cv2.circle(imgcolor, (255+m_j, 170+m_i), 4, (255,0,255), -1)
        #-----------------------------------------------------------------------------------------------------------------
        cv2.putText(imgcolor, 'Xb', (300, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 4)                
        cv2.line(imgcolor, (310-15*6, 95), (310 + 15*6, 95), (0, 0, 0), 4)
        
        
        cv2.putText(imgcolor, 'Yb', (300, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 4)                
        cv2.line(imgcolor, (310-15*6, 125), (310 + 15*6, 125), (0, 0, 0), 4)
        
        
        cv2.putText(imgcolor, 'Zb', (300, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 4)                
        cv2.line(imgcolor, (310-15*6, 155), (310 + 15*6, 155), (0, 0, 0), 4)
        
        #-----------------------------------------------------------------------------------------------------------------
        cv2.putText(imgcolor, 'Xb', (300, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.line(imgcolor, (310, 95), (310 + 30*int(V_For), 95), (0, 255, 0), 2)
        cv2.putText(imgcolor, str(round(V_For,2)), (310 + 35*int(V_For), 95), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 4)
        cv2.putText(imgcolor, str(round(V_For,2)), (310 + 35*int(V_For), 95), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        cv2.putText(imgcolor, 'Yb', (300, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.line(imgcolor, (310, 125), (310 + 30*int(V_Hor), 125), (0, 255, 0), 2)
        cv2.putText(imgcolor, str(round(V_Hor,2)), (310 + 35*int(V_Hor), 125), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 4)
        cv2.putText(imgcolor, str(round(V_Hor,2)), (310 + 35*int(V_Hor), 125), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        cv2.putText(imgcolor, 'Zb', (300, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.line(imgcolor, (310, 155), (310 + 30*int(V_Ver), 155), (0, 255, 0), 2)
        cv2.putText(imgcolor, str(round(V_Ver,2)), (310 + 35*int(V_Ver), 155), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 4)
        cv2.putText(imgcolor, str(round(V_Ver,2)), (310 + 35*int(V_Ver), 155), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # Show info. in depth frame
        cv2.putText(depth, txt1, (257+m_j, 168+m_i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0,255), 2)
        cv2.circle(depth, (255+m_j, 170+m_i), 4, (255,0,255), -1)
        
        # Reset tag_time
        tag_time = 0
    else:
        str_tag = str(tag_time)
        # Show info. in frame
        cv2.putText(imgcolor, 'CLEAR', (300, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        #cv2.putText(imgcolor, str_tag, (300, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if tag_time<5:
            client.moveToPositionAsync(wp[wp_i][0], wp[wp_i][1], GPS[2], velocity=velocity, yaw_mode=airsim.YawMode(False, Heading))
        else:
            client.moveToPositionAsync(wp[wp_i][0], wp[wp_i][1], wp[wp_i][2], velocity=velocity, yaw_mode=airsim.YawMode(False, Heading))
            #tag_time = 0
    GPS = GetPos()
    # Check if reach the waypoint(x,y)
    #if (abs(GPS[0] - wp[wp_i][0]) <= 0.15) and (abs(GPS[1] - wp[wp_i][1]) <= 0.15):
    if math.sqrt(abs(GPS[0] - wp[wp_i][0])**2 + abs(GPS[1] - wp[wp_i][1])**2) <= 2:
        SEND = True
        gps_temp = GetPos()
        client.moveByVelocityAsync(vx=0, vy=0, vz=0, duration=1).join()
        print(">> Arrived at wp{Nwp:}({x:}, {y:}, {z:})!".format(Nwp=wp_i+1, x=gps_temp[0], y=gps_temp[1], z=gps_temp[2]))
        wp_i += 1        
    # Frame show 
    cv2.imshow("Cam", depth)
    cv2.imshow("COLOR", imgcolor)  
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        cv2.destroyAllWindows()
        break
print('\n ==== Mission Complete ! ====')
print(">> LAND")
cv2.destroyAllWindows()
#client.landAsync().join()
client.landAsync()
while 1:
    GPS = GetPos()  
    if abs(GPS[2]) < 1*0.1:        
        break    
    if os.path.exists(path):
        f= open(path,"a+")
    else:
        f= open(path,"w+")                    
    try:            
        f.write(str(GPS[0])+' '+str(GPS[1])+' '+str(GPS[2])+' \n')    
        f.close()  
        time.sleep(0.5)   
    except Exception as e:
        print('error: ',e)        
        pass
client.armDisarm(False)
# Reset
""" print("\n ==== Reset Vehicle ====")
for sec in reversed(range(1,6)):
    print(">> {s:} second left.".format(s=sec))
    time.sleep(1) """
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
