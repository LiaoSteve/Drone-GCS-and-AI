" -------------------------------- SENT DRONE STATUS TO SEVER ------------------------------"
from datetime import datetime
import time, json, sys
import socket

class GCS():
    def __init__(self, ip, port):        
        # ------------------------- udp socket  -------------------------
        self.gcs_ip = gcs_ip
        self.gcs_port = gcs_port
        self.gcs_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)      
        # ------------------------ drone status -------------------------
        self.cur_pos = None
        self.velocity = None
        self.attitude = None
        self.groundspeed = None
        self.mode = None
        self.dist_to_home = None
        self.heading = None
        self.gps_status = None
        self.battery = None  
        # ------------------------waypoint and drone_home -----------------
        self.home = None
        self.wp   = None 
        
    def send_data_to_MapServer(self, message):
        try:           
            self.gcs_sock.sendto(message.encode('utf-8'),(self.gcs_ip,self.gcs_port))            
        except Exception as e:
            print(e)          

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
            data['latitude'] = self.wp[i][1]
            data['longitude'] = self.wp[i][0]       
            data['waypoint'] = i+1
            message = json.dumps(data)            
            self.send_data_to_MapServer(message)                                                   
            i+=1    

    def send_drone_status_to_GCS(self):        
        self.cur_pos = self.vehicle.location.global_relative_frame
        self.velocity = self.vehicle.velocity
        self.attitude = self.vehicle.attitude        
        data = {}
        data['channel']  = '00009'              

        data['velocity_N']     = self.velocity[0]
        data['velocity_E']     = self.velocity[1]
        data['velocity_Down']  = self.velocity[2]
        
        data['latitude']       = self.cur_pos.lat
        data['longitude']      = self.cur_pos.lon    
        data['altitude']       = self.cur_pos.alt

        data['yaw']            = self.attitude.yaw * 57.295 # [radian] to [degree]
        data['roll']           = self.attitude.roll * 57.295
        data['pitch']          = self.attitude.pitch * 57.295

        data['groundspeed']    = self.vehicle.groundspeed 
        data['mode']           = self.vehicle.mode.name        
        data['dist_to_home']   = self.dist_to_home
        #haversine( pos1 = self.cur_pos, pos2_lon = self.home.lon , pos2_lat = self.home.lat)
        data['heading']        = self.vehicle.heading
        data['gps_status']     = str(self.vehicle.gps_0)
        data['battery']        = str(self.vehicle.battery)
        message = json.dumps(data)    
        self.send_data_to_MapServer(message)    

    def send_current_mark(self):       
        data = {}
        #self.cur_pos = self.vehicle.location.global_relative_frame    
        data['channel'] = '00002'
        data['latitude'] = self.cur_pos.lat
        data['longitude'] = self.cur_pos.lon
        message = json.dumps(data)
        self.send_data_to_MapServer(message)          
                          



   