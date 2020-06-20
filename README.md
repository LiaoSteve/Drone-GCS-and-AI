# Drone-GCS-and-AI

- [x] Use yolov3 to detect coast's debries : plastic bottles , plastic bags , caps etc
- [x] RTK
- [x] Obstacle avoidance
- [x] django GCS

***[See demo video 1](https://drive.google.com/open?id=1H6hfDKPacrxpqa3XGIi3flVTbuDY8Ano)***

***[See demo video 2](https://drive.google.com/open?id=1Lma_kVY98y7Zlkeo5f46-ZTPgNvyxGDc)***

***[PID parameter AutoTuning](https://drive.google.com/open?id=12vV1WJXvEIu-ZyxeH2k5318cdNbjx9j2)***


# YOLOv3

**Notice that in model_data folder:**

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
```bush
https://drive.google.com/open?id=1a_9UqMma-1tFE4DrE3QfoBw81_i-6OJM
```

# Airsim
- [x] In AirSim simulator, use the fuzzy rules to control the drone.
- [x] Use depth to sense the objects.

### Author
- [x] Created by Tzung-Hsien Huang, and adapted by LisoSteve
### Test A [ (See demo video)](https://drive.google.com/open?id=1oGbn28wQA_o-EyqqzDqoxLLuQZKde3WK):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_A.gif)

### Test B [ (See demo video)](https://drive.google.com/open?id=1G7rWvAg8GuQ7e9GqmgrFNUia2IaKj6rS):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_B.gif)

### Test F [ (See demo video)](https://drive.google.com/open?id=1KNb6ggzH0gUVQc07_ZdVgUQq8zr_T9sn):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_F.gif)

### Test G [ (See demo video)](https://drive.google.com/open?id=1Bwu4uhnbphOmDBdiDrkOj26QjXSGpqfg):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_G.gif)


# Drone (ardupilot firmware) using RTK (real time kinematic):
## Via Radio Telemery (COM PORT):
### BASE (your notebook mission planner):
* 0. Ref: ***https://ardupilot.org/copter/docs/common-here-plus-gps.html***
* 1. Open Mission planner Initial Setup >> Optional Hardware >> RTK GPS Inject screen
* 2. Connect your F9P to computer via COM PORT and Baud rate is 115200
* 3. Set surveyin accuracy in meter and time in second (you can use default setting: 2(m) and 60(s)) 
* 4. Press connect buttom and wait for BASE is ready

### ROVER (pixhawk which use ardupilot firmware version higher than 3.6.9, i use 3.6.11 hexacopter):
1. Use I2C provide power (that is, plug the GPS module's GPS pin +5V and GND and insert to I2C)
2. Use your F9P pins insert to your pixhawk GPS pins
3. Use mission planner connect to drone's via radio telemery
4. wait for rtk ready
5.(option) paramter list setting: EK2_ALT_SOURCE:2, EK2_POSNE_M_NSE : 0.1

## License 

- [x] Notice that our License is reserved








