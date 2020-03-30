from ObsRS import *
from FlightFunc import *
import FuzzyControlA, FuzzyControlB
import dronekit_sitl

"---------------- Fuzzy System -----------------"
fzA = FuzzyControlA.FuzzyControl()
fzB = FuzzyControlB.FuzzyControl()
fzA.pre_fzprocess()
fzB.pre_fzprocess()
"---------------- Copter -----------------"
connection_string = 'tcp:127.0.0.1:5762'
vehicle = connect(connection_string, wait_ready=True, baud=115200) 
#time.sleep(20)
arm_and_takeoff(vehicle, 10)
"---------------- Realsense Cam -----------------"
c1 = [(170, 100)]
c2 = [(425, 325)]
cent_x = (c1[0][0]+c2[0][0])//2
cent_y = (c1[0][1]+c2[0][1])//2   
Realsense = SenseObstacle(coord1=c1, coord2=c2)
Realsense.start()
time.sleep(2)
"---------------- Main Loop -----------------"
while 1:
    frame_out = Realsense.get_frame()

    if frame_out.any():
        x_location, y_location, z_distance = Realsense.getObsLocation()    
    cv2.rectangle(frame_out, c1[0], c2[0], (0, 255, 255), 2)

    if x_location != np.inf:
        V_Hor, V_Ver, V_For, cost_time = fzA.fzprocess(delta_x=x_location, delta_y=y_location, distance=z_distance)
        send_velocity(vehicle, V_For, V_Hor, 0*-V_Ver)        
        print('\n>> x_:{x}, y_:{y}, distance:{z:.2f}cm'.format(x=x_location, y=y_location, z=z_distance))
        print(">> Defuzzy data: VH= {VH:}, VV= {VV:}, VF= {VF:}".format(VH=V_Hor, VV=V_Ver, VF=V_For))
        cv2.circle(frame_out, (cent_x-x_location, cent_y-y_location), 5, (0,0,255), -1)
    else:
        send_velocity(vehicle, 0.3, 0, 0)
        print('clear')
    result = 'Distance:{z:.2f}cm'.format(z=z_distance)
    cv2.putText(img=frame_out,text=result,org=(175,360),fontFace=cv2.FONT_HERSHEY_DUPLEX,color=(0, 255, 255),fontScale=1) 
    cv2.imshow('realsense',frame_out)
    if cv2.waitKey(1) & 0xFF==27: # esc
        vehicle.close()
        break

