
from cam import *
from sent_images_to_socket_server import *
from gcs import *

import time, timeit, sys, math, json, os
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
from math import asin, sin, cos, sqrt, radians

class DroneSystem(Cam, CamSocket, GCS):
    def __init__(self, 
                 URL = 0,
                 cam_ip = '140.121.130.133', cam_port = 9998,
                 gcs_ip = '140.121.130.133', gcs_port = 9999,
                 connection_string= 'tcp:127.0.0.1:5762', baud = 115200, wait_ready = True, timeout = 180, heartbeat_timeout = 180):
        Cam.__init__(self, URL = URL)  
        CamSocket.__init__(self, ip = cam_ip, port = cam_port)  
        GCS.__init__(self, ip = gcs_ip, port = gcs_port)
        # ----------------------- create my vehicle class ------------
        self.vehicle = connect(ip = connection_string, baud=baud, wait_ready= wait_ready, timeout=timeout, heartbeat_timeout=heartbeat_timeout)
                
        
    # ---------------------------------------- BASIC FUNCTIONS -----------------------------------
    def read_json(self, path):
        # READ COORDINATES FROM JSON 
        input_file = open(path)
        json_array = json.load(input_file)
        coordinates = json_array['features'][0]['geometry']['coordinates']
        return coordinates      
    def haversine(self, pos1, pos2_lon, pos2_lat):
        """
        Get the distance between two positions (note that inputs are in decimal degree)
        pos1 : current point
        pos2 : Reference point 
        """
        lon1, lat1, lon2, lat2 = map(radians, [pos1.lon, pos1.lat, pos2_lon, pos2_lat])
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a)) 
        r = 6378137.0 #-- Radius of "spherical" earth in meter
        return c * r   
    # ----------------------------------------- DRONE CONTROL --------------------------------
    def arm_and_takeoff(self, aTargetAltitude=5):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
        # Wait until the vehicle reaches a safe height before processing the goto
        #  (otherwise the command after Vehicle.simple_takeoff will execute
        #   immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
    def goto(self, location, groundspeed=0.5):
        self.vehicle.simple_goto(location, groundspeed=groundspeed)        
    def close_vehicle(self):
        self.vehicle.close()
    def send_velocity(self, velocity_x, velocity_y, velocity_z):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)
        """ for i in range(duration):
            vehicle.send_mavlink(msg)
            time.sleep(1) """
        self.vehicle.flush()    
    def servo_pwm(self, servo_num, pwm):
        """
        Enable the servo
        """
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_num,
            pwm,
            0,
            0,
            0, 0, 0)
        self.vehicle.send_mavlink(msg)    
    def send_ned_velocity(self, n, e, d, duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        n, e, d, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        for i in range(duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)
        self.vehicle.flush()    
    def security_lock(self, channel):
        while self.vehicle.channels[channel] > 1500 :
            print ("RC_%s :%s" %channel,self.vehicle.channels[channel])
            print ("switch down to continue the mission")
            time.sleep(1)	
        print ("Start Mission")    
    def stop_monitor(self):
        # Not success yet
        while True:
            if self.vehicle.channels['8'] > 1500:
                print("Stop mission ,RTL")
                self.vehicle.mode = VehicleMode("RTL")
                sys.exit(0)    
    def get_attributes(self):
        print (self.vehicle.location.local_frame)    #NED
        print (self.vehicle.attitude)
        print ("Velocity: %s" % self.vehicle.velocity)
        print (self.vehicle.gps_0)
        print ("Groundspeed: %s" % self.vehicle.groundspeed)
        print ("Airspeed: %s" % self.vehicle.airspeed)
        print ( self.vehicle.gimbal)
        print (self.vehicle.battery)
        print ("EKF OK?: %s" % self.vehicle.ekf_ok)
        print ("Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print (self.vehicle.rangefinder)
        print ("Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        print ("Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
        print ("Heading: %s" % self.vehicle.heading)
        print ("Is Armable?: %s" % self.vehicle.is_armable)
        print ("System status: %s" % self.vehicle.system_status.state)

if __name__ =='__main__':
    import dronekit_sitl    
    X_ground = {'lat':25.149657404957285 ,'lon':121.77688926458357}
    sitl = dronekit_sitl.start_default(lat = X_ground['lat'], lon = X_ground['lon'])
    connection_string = sitl.connection_string()      
    drone = DroneSystem(URL=0,cam_ip='140.121.130.133',cam_port=9998, connection_string=connection_string, baud=115200, wait_ready=True)
    drone.cam_start()
    drone.cam_socket_connect()
    drone.arm_and_takeoff(10)
    wp = drone.read_json('data/X_ground.json') 
    i=2
    target = LocationGlobalRelative(lat=wp[i][1]+1,lon=wp[i][0],alt=10)    
    while 1:
        
        drone.goto(target, groundspeed=10)
        frame = drone.cam_getframe()          
        if not len(frame): # wait for frame
            continue   
        
        drone.sent_img_to_server(frame)
        
        cv2.imshow('',frame)        
        key = cv2.waitKey(1) & 0xFF
        if key==27:
            drone.cam_stop()
            drone.close_vehicle()
            break
    
