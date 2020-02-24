# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode


import airsim
import threading
import cv2
import time
import sys
import numpy as np 
import tkinter as tk
from math import cos,sin,radians
import pprint

cur_yaw = 0 # heading :+x axis
# Fly given velocity vector for 5 seconds
duration = 0.05
speed = 1.0
delay = duration * speed
#i can
# using airsim.DrivetrainType.MaxDegreeOfFreedom means we can control the drone yaw independently
# from the direction the drone is flying.  I've set values here that make the drone always point inwards
# towards the inside of the box (which would be handy if you are building a 3d scan of an object in the real world).
def frd2ned_in_velocity(theta,v_front,v_right):       
    v_frd = np.array([v_front,v_right]).reshape(2,1)
    rotation_matrix = np.array([cos(radians(theta)),sin(radians(theta)),-sin(radians(theta)),cos(radians(theta))]).reshape(2,2)
    v_ned = np.dot(rotation_matrix,v_frd).reshape(-1,)
    print('------------------------------------------')
    print('v_ned: ',v_ned)
    return v_ned   

class LidarTest:
    def __init__(self):
        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        home = self.client.getMultirotorState().kinematics_estimated.position
        print('home: ',home)
    def key(self,event):
        global cur_yaw
        z  = self.client.getMultirotorState().kinematics_estimated.position.z_val
        if event.char == event.keysym: #-- standard keys                
            if event.keysym == 's':
                self.client.moveByVelocityZAsync(0,0,z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, cur_yaw)).join()            
            if event.keysym == 'u':
                self.client.moveByVelocityZAsync(0,0,z-1,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, cur_yaw)).join()            
            if event.keysym == 'd':
                self.client.moveByVelocityZAsync(0,0,z+1,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, cur_yaw)).join()            
            if event.keysym == 'l':
                self.client.landAsync().join()
            if event.keysym == 't':
                print('take off') 
                self.client.takeoffAsync().join()
            # turn right or left
            if event.keysym == 'q':
                cur_yaw = cur_yaw - 10
                print('cur_yaw: ',cur_yaw)
                self.client.moveByVelocityZAsync(0,0,z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, cur_yaw)).join()  
            if event.keysym == 'e':
                cur_yaw = cur_yaw + 10
                print('cur_yaw: ',cur_yaw)
                self.client.moveByVelocityZAsync(0,0,z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, cur_yaw)).join()  
            if event.keysym == 'p':
                p = self.client.getMultirotorState().kinematics_estimated.position            
                print("[x,y,z] = [{:.6f}, {:.6f}, {:.6f}]".format(p.x_val,p.y_val,p.z_val))
            # get data
            if event.keysym == 'v':
                print(self.client.getVelocity())
            if event.keysym == 'o':
                print(self.client.getOrientation())
            if event.keysym == 'g':            
                g = self.client.getMultirotorState().gps_location
                print("[lat,lon,alt] = [{:.6f}, {:.6f}, {:.6f}]".format(g.latitude,g.longitude,g.altitude))
            if event.keysym == 'r':
                self.client.reset()

        else: #-- non standard keys
            z  = self.client.getMultirotorState().kinematics_estimated.position.z_val
            if event.keysym == 'Up':
                vx = speed
                vy = 0                                  
                V = frd2ned_in_velocity(-cur_yaw,vx,vy) 
                self.client.moveByVelocityZAsync(V[0],V[1],z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, cur_yaw)).join()
                time.sleep(delay)
            elif event.keysym == 'Down':
                vx = -speed
                vy = 0                      
                V = frd2ned_in_velocity(-cur_yaw,vx,vy) 
                self.client.moveByVelocityZAsync(V[0],V[1],z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, cur_yaw)).join()
                time.sleep(delay)
            elif event.keysym == 'Left':
                vx = 0
                vy = -speed            
                V = frd2ned_in_velocity(-cur_yaw,vx,vy) 
                self.client.moveByVelocityZAsync(V[0],V[1],z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, cur_yaw)).join()
                time.sleep(delay)
            elif event.keysym == 'Right':
                vx = 0
                vy = speed            
                V = frd2ned_in_velocity(-cur_yaw,vx,vy) 
                self.client.moveByVelocityZAsync(V[0],V[1],z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, cur_yaw)).join()
                time.sleep(delay) 

    def execute(self):   

        while 1:
            lidarData = self.client.getLidarData()
            if (len(lidarData.point_cloud) < 3):
                print("No points received from Lidar data")
            else:
                points = self.parse_lidarData(lidarData)
                print("Reading : time_stamp: %d number_of_points: %d" % ( lidarData.time_stamp, len(points)))
                print("----------------------------------------------")
                print("lidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                print("lidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
            time.sleep(1)

    def parse_lidarData(self, data):
        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))       
        return points

    def write_lidarData_to_disk(self, points):
        # TODO
        print("not yet implemented")

    def stop(self):
        airsim.wait_key('Press any key to reset to original state')
        self.client.armDisarm(False)
        self.client.reset()
        self.client.enableApiControl(False)
        print("Done!\n") 
if __name__ == '__main__':
    try :
        # connect to client         
        
        
        # create Lidar class
        drone_manual_control = LidarTest()
        #lidar_job = threading.Thread(target=drone_manual_control.execute)
        
        #lidar_job.start()
        #tkinter
        root = tk.Tk()  
        root.bind_all('<Key>', drone_manual_control.key)
        root.mainloop() 

    except Exception as e:
        print(e)
    finally:
        sys.exit(0)
