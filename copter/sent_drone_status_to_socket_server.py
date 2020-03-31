" -------------------------------- SENT DRONE STATUS TO SEVER ------------------------------"
from datetime import datetime
import time, json, sys
import socket
import cv2
from FlightFunc import haversine, read_json 

class DRONE_STATUS_SOCKET(object):
    def __init__(self, vehicle, home, wp, ip, port):
        self.vehicle = vehicle
        self.ip = ip
        self.port = port 
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(1)      
        self.home = home 
        self.wp   = wp
    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(0.1)   
            self.sock.connect((self.ip,self.port))
            print('Success connecting to Mapserver ')
        except :            
            print('>> Error to connect to Map_server\nmaybe should open Map page or Map server')             
    def send_data_to_MapServer(self, message):
        try:           
            self.sock.send(message.encode('utf-8'))
        except:
            print('\n*******\nSend data to MapServer error')
            try:                
                print('Try connecting to MapServer ...')
                self.connect()
                self.mark_vehicle_home()  
                self.generate_checkpoint()  
                print('OK')                                         
            except:
                print('>> error!! maybe you should open GCS Map ...')                
    def mark_vehicle_home(self):
        data = {}
        data['channel'] = '00004'
        data['latitude'] = self.home.lat
        data['longitude'] = self.home.lon
        message = json.dumps(data)
        print(message)
        time.sleep(0.1)
        self.send_data_to_MapServer(message)          
    def generate_checkpoint(self): 
        """Mark the copter waypoints on the map, and
           use geojson.io to build a '.json' file please."""               
        data = {}  
        data['channel'] = '00003'         
        i = 0
        time.sleep(0.1)
        while i < len(self.wp):        
            data['latitude'] = self.wp[i][1]
            data['longitude'] = self.wp[i][0]       
            data['waypoint'] = i+1
            message = json.dumps(data)
            print(message)
            self.send_data_to_MapServer(message)                                                   
            i+=1    
    def send_drone_status_to_GCS(self):        
        cur_pos = self.vehicle.location.global_relative_frame
        velocity = self.vehicle.velocity
        attitude = self.vehicle.attitude        
        data = {}
        data['channel']  = '00009'              

        data['velocity_N']     = velocity[0]
        data['velocity_E']     = velocity[1]
        data['velocity_Down']  = velocity[2]

        data['latitude']       = cur_pos.lat
        data['longitude']      = cur_pos.lon    
        data['altitude']       = cur_pos.alt

        data['yaw']            = attitude.yaw
        data['roll']           = attitude.roll
        data['pitch']          = attitude.pitch

        data['groundspeed']    = self.vehicle.groundspeed 
        data['mode']           = self.vehicle.mode.name        
        data['dist_to_home']   = haversine( pos1 = cur_pos, pos2_lon = self.home.lon , pos2_lat = self.home.lat)
        data['heading']        = self.vehicle.heading
        data['gps_status']     = str(self.vehicle.gps_0)
        data['battery']        = str(self.vehicle.battery)
        message = json.dumps(data)    
        self.send_data_to_MapServer(message)        

        # send_current_mark
        ''' data = {}        
        data['channel'] = '00002'
        data['latitude'] = cur_pos.lat
        data['longitude'] = cur_pos.lon
        message = json.dumps(data)
        self.send_data_to_MapServer(message) ''' 

    def send_current_mark(self):       
        data = {}
        cur_pos = self.vehicle.location.global_relative_frame    
        data['channel'] = '00002'
        data['latitude'] = cur_pos.lat
        data['longitude'] = cur_pos.lon
        message = json.dumps(data)
        self.send_data_to_MapServer(message)          
        #return cur_pos
        
           


if __name__ == '__main__':
    import dronekit    
    import dronekit_sitl
    from FlightFunc import arm_and_takeoff

    X_ground = {'lat':25.149657404957285 ,'lon':121.77688926458357}
    sitl = dronekit_sitl.start_default(lat = X_ground['lat'], lon = X_ground['lon'])
    connection_string = sitl.connection_string()       
    my_drone = dronekit.connect(connection_string, wait_ready=True, baud=115200, heartbeat_timeout=180, timeout=180)
    home = my_drone.location.global_relative_frame
    wp = read_json('data/X_ground.json') 

    my_drone_status_socket = DRONE_STATUS_SOCKET(my_drone, home, wp, '140.121.130.133', 9999)
    my_drone_status_socket.connect()
    my_drone_status_socket.mark_vehicle_home()
    my_drone_status_socket.generate_checkpoint()

    arm_and_takeoff(my_drone,10)
    i = 1
    target = dronekit.LocationGlobalRelative(lat=wp[i][1]+1,lon=wp[i][0],alt=3)
    my_drone.simple_goto(target, groundspeed=3) # max groundspeed 15[m/s] in dronekit sitl 
    
    while 1:  
        my_drone_status_socket.send_current_mark()              
        my_drone_status_socket.send_drone_status_to_GCS()                    
        time.sleep(0.5)