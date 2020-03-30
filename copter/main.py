"-------------------------- MAIN DRONE CONTORL SCRIPT------------------------"
from yolo import*
from cam import*
from sent_images_to_socket_server import*

my_cam  = CAM(URL = 0)
my_cam.start()
time.sleep(2)

my_yolo = YOLO()
my_yolo.start()

my_cam_socket = CAM_SOCKET('140.121.130.133',9998)
my_cam_socket.connect()

while 1:
    frame = my_cam.getframe()
    image = Image.fromarray(frame)

    image = my_yolo.detect_image(image)
    result = np.asarray(image)    

    cv2.imshow("result", result)   

    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        my_cam.stop()
        my_yolo.close_session()
        cv2.destroyAllWindows()
        break    
        
        
