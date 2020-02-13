from dronekit import connect
import dronekit_sitl
import socket

def globals_vehicle_init():
    """
    USE dronkit-sitl :
    dronekit-sitl copter --home=51.503667218218546,-0.10445594787597656,584,353 (lat,lon,alt,yaw)
    """
    global vehicle
    global Home
    print('---------------    SITL   ---------------------')          
    #Fishing_port = {'lat': 25.148266188681344,'lon':121.78979873657227}
    X_ground = {'lat':25.149657404957285 ,'lon':121.77688926458357}
    sitl = dronekit_sitl.start_default(lat = X_ground['lat'], lon = X_ground['lon'])
    
    #connection_string = 'tcp:127.0.0.1:5760'
    
    connection_string = sitl.connection_string()        
    '''connection_string = 'COM4'
    vehicle = connect(connection_string, baud=57600, wait_ready = True )'''

    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True, baud=115200) 
    #vehicle = connect('COM3', baud=57600, wait_ready = True, heartbeat_timeout=180, timeout=180)
    Home    = vehicle.location.global_relative_frame
    print('Home :',Home)

def object_identify_init():
    global trash_num 
    global cap_num 
    trash_num = 0
    cap_num =0

def socket_connect_init():
    global sock
    #while 1:
    try:
        print('trying to connect to MapServer ...')
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        sock.connect(('140.121.130.133',9999))
        print('Success connecting to Mapserver ')
        #break
    except Exception as e:                
        print('mapserver',e)
