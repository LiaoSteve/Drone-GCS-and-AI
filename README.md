# Drone-GCS-and-AI
## Demo

* ***1. [Use yolov3 to detect coast's debries : plastic bottles , plastic bags , caps etc](https://drive.google.com/file/d/15HNY2gMDhPa_sieLgT28_APcT03e1TX1/view?usp=sharing)***

* ***2. [RTK](https://drive.google.com/file/d/1jjvXl_TojcVaHIFGIpz3gNLZ7TZ8hiRM/view?usp=sharing)***

* ***3. Obstacle avoidance***

* ***4. django GCS***

* ***5. [See demo video 1](https://drive.google.com/open?id=1H6hfDKPacrxpqa3XGIi3flVTbuDY8Ano)***

* ***6. [See demo video 2](https://drive.google.com/open?id=1Lma_kVY98y7Zlkeo5f46-ZTPgNvyxGDc)***

* ***7. [PID parameter AutoTuning](https://drive.google.com/open?id=12vV1WJXvEIu-ZyxeH2k5318cdNbjx9j2)***


## YOLOv3

**Notice that in copter/model_data folder:**

```
model_data/
            trained_weights_final_009.h5
            yolo_anchors_009.txt
            voc_classes.txt 
```

- [x] You should download ***trained_weights_final_009.h5*** below, and add to ***model_data*** folder
``` 
https://drive.google.com/open?id=1QVF2AbILUvDLGh02Uwbuzf-lKC-3xqI2
```
 
- [x] Download video ***20191010.mov*** below
```
https://drive.google.com/open?id=1a_9UqMma-1tFE4DrE3QfoBw81_i-6OJM
```

- [x] connect your webcam, and run copter/yolo.py

## Airsim
* Created by Tzung-Hsien Huang, and adapted by LisoSteve
* In AirSim simulator, use the fuzzy rules to control the drone.
* Use depth to sense the objects.

* Test A [ (See demo video)](https://drive.google.com/open?id=1oGbn28wQA_o-EyqqzDqoxLLuQZKde3WK):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_A.gif)

* Test B [ (See demo video)](https://drive.google.com/open?id=1G7rWvAg8GuQ7e9GqmgrFNUia2IaKj6rS):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_B.gif)

* Test F [ (See demo video)](https://drive.google.com/open?id=1KNb6ggzH0gUVQc07_ZdVgUQq8zr_T9sn):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_F.gif)

* Test G [ (See demo video)](https://drive.google.com/open?id=1Bwu4uhnbphOmDBdiDrkOj26QjXSGpqfg):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_G.gif)

## How to getting started with ardupilot and pixhawk : 
### First setup
1. Upload firmware : use [mission planner](https://ardupilot.org/planner/docs/mission-planner-installation.html) upload firmware, and upload chibiOS, i use hexacopter version 3.6.11.
2. Connect your pixhawk with mission planner, and open Intial setup then choose your UAV frame.
3. [Accelerometer Calibration](https://ardupilot.org/copter/docs/common-accelerometer-calibration.html?highlight=calibration).
4. [Radio Control Calibration](https://ardupilot.org/copter/docs/common-radio-control-calibration.html?highlight=calibration).
5. [Compass Calibration](https://ardupilot.org/copter/docs/common-compass-calibration-in-mission-planner.html?highlight=calibration) : if you use GPS module, you only need to use compass 1 (external compass). After calibration, reboot your pixhawk.
6. ESC Calibration: Push throttle to highest position, power up pixhawk and wait for RGB LED. Reboot, disarm your hardware switch and wait for beep~ beep~ beep~. Push throttle to lowest position, and wait for beep~ beep~ beep~. Push throttle from lowest position to highest position, and push throttle to lowest position. Reboot your pixhawk.

### PID auto tuning
* Tuning   : ***https://ardupilot.org/copter/docs/tuning.html***
* Autotune : ***https://ardupilot.org/copter/docs/autotune.html***
1. Open mission planner and connect your pixhawk via serial port.
2. Choose Config/Tune panel>> Extended Tuning >> Alttitude Hold >> `RC7 opt` >> `Auto tune`, and `RC10 opt` >> `Motor Interlock`
3. Config/Tune >> Full Parameter >> set `AUTOTUNE_AGGR` : 0.05 (default 0.1),  `AUTOTUNE_AXES`: 0 (I suggest tuning roll, pitch, yaw separately)
4. fly in reality: take off in alt-hold flight mode, and push throttle to 50% ~ 59%, and activate your Radio Controller CH7 to HIGH, then PID Auto tuning will work. Wait UAV beep~ indicate that autotuning is done, and let your UAV land in Alt-Hold fight mode, and arm(You will see pid parameters saved). Finally, disabled your CH7 to LOW.

### UAV auto fight mode ([RTL](https://ardupilot.org/copter/docs/rtl-mode.html), [LAND](https://ardupilot.org/copter/docs/land-mode.html), [LOITER](https://ardupilot.org/copter/docs/loiter-mode.html), [AUTO](), [GUIDED](https://ardupilot.org/copter/docs/ac2_guidedmode.html), [ALT-HOLD](https://ardupilot.org/copter/docs/altholdmode.html#altholdmode-controls), [brake](https://ardupilot.org/copter/docs/brake-mode.html))
#### 1. Open mission planner and connect to your pixhawk, then choose Config/Tune panel>> Full parameter list >> search WPN :
* `WPNAV_SPEED ` : if set `RTL_speed` to zero, RTL horizontal velosity will use this speed .
* `WPNAV_RADIUS `: in AUTO mode, if this parameter set 100, that is, once your UAV enter waypoint radius 100 cm, pixhawk will stop and go to the next waypoint. 
* `WPNAV_SPEED_UP ` : Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission.
* `WPNAV_SPEED_DN`  : Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission.
#### 2. Open mission planner and connect to your pixhawk, then choose Config/Tune panel>> Full parameter list >> search SPEED :
* `LAND_SPEED` : The descent speed for the final stage of landing in cm/s.
* `LAND_SPEED_HIGH` : The descent speed for the first stage of landing in cm/s. If this is zero then WPNAV_SPEED_DN is used.
* `RTL_SPEED` : Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WPNAV_SPEED will be used instead.
* `LOIT_SPEED` : Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode.
* `PILOT_SPEED_DN` : The maximum vertical descending velocity the pilot may request in cm/s.
* `PILOT_SPEED_UP` : The maximum vertical ascending velocity the pilot may request in cm/s.
#### 3. other parameters :
* `RTL_ALT` : The minimum alt above home the vehicle will climb to before returning.  If the vehicle is flying higher than this value it will return at its current altitude.
* `WP_YAW_BEHAVIOR` : Determines how the autopilot controls the yaw during missions and RTL
* `MOT_SPIN_MIN` : Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range.  Should be higher than MOT_SPIN_ARM.
* 
### 4. some issue :
* [Configuring a Telemetry Radio using Mission Planner](https://ardupilot.org/copter/docs/common-configuring-a-telemetry-radio-using-mission-planner.html#common-configuring-a-telemetry-radio-using-mission-planner)
* [Telemetry Radio](https://ardupilot.org/copter/docs/common-sik-telemetry-radio.html#common-sik-telemetry-radio)
* [Setting Hover Throttle](https://ardupilot.org/copter/docs/ac_throttlemid.html#ac-throttlemid)
* [Non-GPS Navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html#common-non-gps-navigation-landing-page)

## How to be a Maker : 
* [Flight Evaluation](https://flyeval.com/)
* [My ZD-850 Hexacopter Design](https://drive.google.com/file/d/1B6qVjqUTX0_z9jWVn1LjFxyjzFRpKL28/view?usp=sharing)
#### ZD 850 hexacopter framen 
* 1. ZD 850 hexacopter frame
* 2. 14*4.7 propeller 
* 3. LA4108B 390KV motor
* 4. HobbyWing eagle 30A
* 5. ACE TATTU Lipo 6S 22.2V
## Drone with RTK :
### ***1. via radio telemery (serial port)***
#### Base (GCS mission planner) 
0. ***https://ardupilot.org/copter/docs/common-here-plus-gps.html***
1. Open Mission planner Initial Setup >> Optional Hardware >> RTK GPS Inject screen
2. Connect your F9P to computer via COM port and baud rate is 115200
3. Check the autoconfig, then set the surveyin accuracy in meter and time in second (you can use default setting: 2(m) and 60(s)) 

#### Rover 
0. Notice that your ardupilot firmware version should higher than 3.6.9, i use 3.6.11 hexacopter.
1. Use I2C provide power (that is, plug the GPS module's GPS pin +5V and GND and insert to I2C).
2. Use your F9P pins insert to your pixhawk GPS pins.
3. Use mission planner connect to drone's via radio telemery.
4. Wait for rtk fixed.
5. (option) paramter list setting: EK2_ALT_SOURCE:2, EK2_POSNE_M_NSE : 0.1[meter] or 0.01[meter](0.01 should use carefully).

### 2. ***via wifi or 4G LTE***
#### Base (tcp server) 
0. Use Windows 10 as server, and you need a Static IP.
1. `git clone https://github.com/tomojitakasu/RTKLIB_bin/tree/rtklib_2.4.3/bin`
2. Click `strsvr.exe`, `Input Type` choose `Serial`, `opt` choose COM port where F9P is connecting and baud rate is 115200, `output Type` choose `TCP server` and give it a port, and click `start` bottom.
3. Open Mission planner Initial Setup >> Optional Hardware >> RTK GPS Inject screen.
4. Connect your F9P to computer via tcp ip and Baud rate is 115200.
5. Set surveyin accuracy in meter and time in second (you can use default setting: 2(m) and 60(s)) 
6. Press connect bottom and wait for BASE is ready.

#### Rover 
0. Notice that your ardupilot firmware version should higher than 3.6.9, i use 3.6.11 hexacopter.
1. Use I2C provide power (that is, plug the GPS module's GPS pin +5V and GND and insert to I2C).
2. Use your F9P pins insert to your pixhawk GPS pins.
3. Use mission planner connect to drone's via radio telemery.
4. wait for rtk fixed.
5. (option) paramter list setting: EK2_ALT_SOURCE:2, EK2_POSNE_M_NSE : 0.1[meter] or 0.01[meter](0.01 should use carefully).

## License 
* Notice that our License is reserved








