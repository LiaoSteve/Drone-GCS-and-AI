"-------------------------- MAIN DRONE CONTORL SCRIPT------------------------"
from yolo import *
from cam import *
from sent_images_to_socket_server import *
from sent_drone_status_to_socket_server import *
from FlightFunc import *

X_ground = {'lat':25.149657404957285 ,'lon':121.77688926458357}
my_drone = connect(ip='tcp:127.0.0.1:5762', wait_ready=True, baud=115200, heartbeat_timeout=180, timeout=180)
home = my_drone.location.global_relative_frame
wp = read_json('data/X_ground.json') 

my_drone_status_socket = DRONE_STATUS_SOCKET(my_drone, home, wp, ip='140.121.130.133', port=9999)
my_drone_status_socket.connect()
my_drone_status_socket.mark_vehicle_home()
my_drone_status_socket.generate_checkpoint()

my_cam  = CAM(URL = 0)
my_cam.start()
time.sleep(2)

#my_yolo = YOLO()
#my_yolo.start()

my_cam_socket = CAM_SOCKET('140.121.130.133',9998)
my_cam_socket.connect()

arm_and_takeoff(my_drone,10)
i = 1
target = dronekit.LocationGlobalRelative(lat=wp[i][1],lon=wp[i][0],alt=3)
my_drone.simple_goto(target, groundspeed=3) # max groundspeed 15[m/s] in dronekit sitl 
while 1:
    frame = my_cam.getframe()
    #image = Image.fromarray(frame)

    #image = my_yolo.detect_image(image)
    #result = np.asarray(image)    

    cv2.imshow("result", frame)   

    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        my_cam.stop()
        #my_yolo.close_session()
        cv2.destroyAllWindows()
        break
        
        
