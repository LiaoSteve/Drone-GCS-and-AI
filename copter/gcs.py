from datetime import datetime
import time, json, sys
from timeit import default_timer as timer
import socket, logging
import threading
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
import numpy as np
from math import asin, sin, cos, sqrt, radians
'''
------------------------------ -----------------------------------------------------
Make sure that our drone is in guided mode :
@  Author  : LiaoSteve.
@  My Goal : create my ground center station throught network (4G, Wi-Fi, Ethernet).
@  Do do list:
            1. resolve GCS map problem : home and waypoint marks, resent to the GCS.
-------------------------------------------------------------------------------------'''
class GCS():
    def __init__(self, gcs_ip, gcs_port, wp_path = 'data/X_ground.json', connection_string= 'tcp:127.0.0.1:5762', baud = 115200, wait_ready = True, timeout = 180, heartbeat_timeout = 180):     
        # -- logger
        self.__gcs_log = logging.getLogger(__name__)            
        self.__gcs_log.setLevel(logging.DEBUG)
        # -- gcs enabled
        self.gcs_isstop = False
        # -- udp socket  
        self.__gcs_ip   = gcs_ip
        self.__gcs_port = gcs_port
        self.__gcs_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
        # -- my drone 
        self.vehicle = connect( ip = connection_string, 
                                baud = baud, 
                                wait_ready = wait_ready, 
                                timeout = timeout, 
                                heartbeat_timeout = heartbeat_timeout)
        self.__gcs_log.info(' >> Connect to pixhawk completely ')
        self.__gcs_log.info(self.vehicle.version) 
        # -- waypoints and drone_home 
        self.home           = self.vehicle.location.global_relative_frame
        self.wp             = self.read_json(wp_path)
        # -- current index of wp
        self.num_wp         = 0     
        self.target         = LocationGlobalRelative(lat=self.wp[self.num_wp][1],lon=self.wp[self.num_wp][0],alt=10)
        self.dist_to_home   = None  
        self.dist_to_target = None       
        # -- drone status 
        self.cur_pos        = None
        self.velocity       = None
        self.attitude       = None
        self.groundspeed    = None
        self.mode           = None        
        self.heading        = None
        # -- GPS info
        self.gps_status     = None
        self.gps_fix        = None
        self.gps_num        = None
        # -- Battery info
        self.batt_vol       = None          
        self.batt_level     = None
        # --Timer 
        
    def next_target(self):      
        if self.num_wp < len(self.wp)-1:                 
            self.num_wp +=1                          
            self.target = LocationGlobalRelative(lat=self.wp[self.num_wp][1],lon=self.wp[self.num_wp][0],alt=10)
        else:
            self.num_wp = 0                         
            self.target = LocationGlobalRelative(lat=self.wp[self.num_wp][1],lon=self.wp[self.num_wp][0],alt=10)
        time.sleep(1)

    def GCS_start(self):        
        threading.Thread(target=self.status_monitor, daemon=True, args=()).start()

    def status_monitor(self):        
        self.__gcs_log.info(' >> GCS monitor started !')
        while (not self.gcs_isstop):
            self.cur_pos      = self.vehicle.location.global_relative_frame # lat, lon, alt
            self.velocity     = self.vehicle.velocity                       # [m/s]
            self.attitude     = self.vehicle.attitude                       # yaw, roll, pitch  in [rad]          
            self.groundspeed  = self.vehicle.groundspeed                    # [m/s]
            self.mode         = self.vehicle.mode.name
            self.heading      = self.vehicle.heading                        # [deg]

            self.batt_vol     = self.vehicle.battery.voltage
            self.batt_level   = self.vehicle.battery.level

            self.gps_status   = self.vehicle.ekf_ok
            self.gps_fix      = self.vehicle.gps_0.fix_type
            self.gps_num      = self.vehicle.gps_0.satellites_visible
                   
            self.dist_to_home = self.haversine( pos1 = self.cur_pos, pos2_lon = self.home.lon , pos2_lat = self.home.lat)
            self.dist_to_target = self.haversine( pos1 = self.cur_pos, pos2_lon = self.target.lon , pos2_lat = self.target.lat)            
            
            self.send_current_mark()
            self.send_drone_status_to_GCS()                           
            time.sleep(1)

    def send_data_to_MapServer(self, message):
        try:           
            self.__gcs_sock.sendto(message.encode('utf-8'),(self.__gcs_ip,self.__gcs_port))            
        except Exception as e:
            self.__gcs_log.info(e)    

    def mark_vehicle_home(self):
        data = {}
        data['channel'] = '00004'
        data['latitude'] = self.home.lat
        data['longitude'] = self.home.lon
        message = json.dumps(data)            
        self.send_data_to_MapServer(message) 

    def generate_checkpoint(self): 
        """Mark the copter waypoints on the map, and
           use geojson.io to build a '.json' file please."""               
        data = {}  
        data['channel'] = '00003'         
        i = 0        
        while i < len(self.wp):                    
            data['latitude']    = self.wp[i][1]
            data['longitude']   = self.wp[i][0]       
            data['waypoint']    = i+1
            message = json.dumps(data)            
            self.send_data_to_MapServer(message)                                                   
            i+=1    
        del i

    def send_drone_status_to_GCS(self):                    
        data = {}
        data['channel']  = '00009'              

        data['velocity_N']     = round(self.velocity[0], 2)
        data['velocity_E']     = round(self.velocity[1], 2)
        data['velocity_Down']  = round(self.velocity[2], 2)
        
        data['latitude']       = self.cur_pos.lat
        data['longitude']      = self.cur_pos.lon    
        data['altitude']       = self.cur_pos.alt
        
        data['yaw']            = round(self.attitude.yaw * 57.295) # [radian] to [degree]
        data['roll']           = round(self.attitude.roll * 57.295)
        data['pitch']          = round(self.attitude.pitch * 57.295)

        data['groundspeed']    = round(self.vehicle.groundspeed, 2)
        data['mode']           = self.mode
        data['dist_to_home']   = round(self.dist_to_home, 2)
        data['dist_to_target'] = round(self.dist_to_target, 2)
        
        data['heading']        = self.heading
        data['gps_status']     = 'ekf_ok:{} fix:{} sat:{}'.format(self.gps_status, self.gps_fix, self.gps_num)
        data['battery']        = 'vol:{} level:{}'.format(self.batt_vol,self.batt_level)
        message = json.dumps(data)    
        self.send_data_to_MapServer(message)   

    def send_current_mark(self):       
        data = {}        
        data['channel'] = '00002'
        data['latitude'] = self.cur_pos.lat
        data['longitude'] = self.cur_pos.lon
        message = json.dumps(data)        
        self.send_data_to_MapServer(message)   

    # ---------------------------------------- BASIC FUNCTIONS -----------------------------------
    def read_json(self, path):
        # READ COORDINATES FROM JSON 
        try:
            input_file = open(path)
            json_array = json.load(input_file)
            coordinates = json_array['features'][0]['geometry']['coordinates']
            return coordinates 
        except Exception as e:
            self.__gcs_log.warning(str(e))
            return None
             
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
        self.__gcs_log.info("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            self.__gcs_log.info(" Waiting for vehicle to initialise...")
            time.sleep(1)
        self.__gcs_log.info("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            self.__gcs_log.info(" Waiting for arming...")
            time.sleep(1)
        self.__gcs_log.info("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
        # Wait until the vehicle reaches a safe height before processing the goto
        #  (otherwise the command after Vehicle.simple_takeoff will execute
        #   immediately).
        while True:
            self.__gcs_log.info(f">> Altitude: {self.vehicle.location.global_relative_frame.alt}")
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                self.__gcs_log.info("Reached target altitude")
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
            self.__gcs_log.warning("RC_%s :%s" %channel,self.vehicle.channels[channel])
            self.__gcs_log.warning("switch down to continue the mission")
            time.sleep(1)	
        self.__gcs_log.warning("Start Mission")  

    def stop_monitor(self):
        # Not success yet
        while True:
            if self.vehicle.channels['8'] > 1500:
                self.__gcs_log.warning("Stop mission ,RTL")
                self.vehicle.mode = VehicleMode("RTL")
                sys.exit(0)    

    def get_attributes(self):       
        self.__gcs_log.info("System status: %s" % self.vehicle.system_status.state)   
        self.__gcs_log.info( self.vehicle.gimbal)        
        self.__gcs_log.info("EKF OK?: %s" % self.vehicle.ekf_ok)
        self.__gcs_log.info("Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        self.__gcs_log.info(self.vehicle.rangefinder)
        self.__gcs_log.info("Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        self.__gcs_log.info("Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)        
        self.__gcs_log.infos("Is Armable?: %s" % self.vehicle.is_armable)
        
                          
if __name__ == '__main__':
    import dronekit_sitl
    X_ground = {'lat':25.149657404957285 ,'lon':121.77688926458357}
    sitl = dronekit_sitl.start_default(lat = X_ground['lat'], lon = X_ground['lon'])
    connection_string = sitl.connection_string()      
    gcs = GCS(gcs_ip = '140.121.130.133', gcs_port = 9999, connection_string = connection_string)
    gcs.GCS_start()
    gcs.arm_and_takeoff(10)
    gcs.mark_vehicle_home()
    gcs.generate_checkpoint()    
    while 1:
        gcs.goto(gcs.target, 3)
        print(round(gcs.dist_to_target, 2))
        if gcs.dist_to_target < 2:
            print('arrive wp:',gcs.num_wp+1)
            gcs.next_target()
            gcs.mark_vehicle_home()
            gcs.generate_checkpoint()
        time.sleep(1)

   