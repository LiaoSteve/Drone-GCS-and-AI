# Drone-GCS-and-AI
*A research diary of a drone system that monitors the coastal pollution*

### -------------------- My Demo video  ---------------
***1. [Use yolov3 to detect coast's debries : plastic bottles , plastic bags , caps etc](https://drive.google.com/file/d/15HNY2gMDhPa_sieLgT28_APcT03e1TX1/view?usp=sharing)***

***2. [RTK](https://drive.google.com/file/d/1jjvXl_TojcVaHIFGIpz3gNLZ7TZ8hiRM/view?usp=sharing)***

***3. Obstacle avoidance***

***4. django GCS***

***5. [See demo video 1](https://drive.google.com/open?id=1H6hfDKPacrxpqa3XGIi3flVTbuDY8Ano)***

***6. [See demo video 2](https://drive.google.com/open?id=1Lma_kVY98y7Zlkeo5f46-ZTPgNvyxGDc)***

***7. [PID parameter AutoTuning](https://drive.google.com/open?id=12vV1WJXvEIu-ZyxeH2k5318cdNbjx9j2)***


## YOLOv3
[![license](https://img.shields.io/github/license/mashape/apistatus.svg)](LICENSE)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3939059.svg)](https://doi.org/10.5281/zenodo.3939059)
* [https://github.com/LiaoSteve/Trash-Dataset-and-object-detection/tree/master](https://github.com/LiaoSteve/Trash-Dataset-and-object-detection/tree/master)

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

- [x] connect your webcam, and run copter/yolo.py

## Airsim
* Created by Tzung-Hsien Huang, and adapted by LisoSteve
* In AirSim simulator, use the fuzzy rules to control the drone.
* Use depth to sense the objects.
#### Citation
```
T. H. Huang, "Path Planning and Visual Obstacle Avoidance Based on Fuzzy Theory for UAV Apllication," 2020 
```
* Test A [ (See demo video)](https://drive.google.com/open?id=1oGbn28wQA_o-EyqqzDqoxLLuQZKde3WK):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_A.gif)

* Test B [ (See demo video)](https://drive.google.com/open?id=1G7rWvAg8GuQ7e9GqmgrFNUia2IaKj6rS):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_B.gif)

* Test F [ (See demo video)](https://drive.google.com/open?id=1KNb6ggzH0gUVQc07_ZdVgUQq8zr_T9sn):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_F.gif)

* Test G [ (See demo video)](https://drive.google.com/open?id=1Bwu4uhnbphOmDBdiDrkOj26QjXSGpqfg):

![image](https://github.com/LiaoSteve/Drone-GCS-and-AI/blob/django_app/airsim/ForAirSim/Data_gif_G.gif)

## Getting started with ardupilot and pixhawk : 
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
2. Choose Config/Tune panel>> Extended Tuning >> Alttitude Hold >> `RC7 opt` >> `Auto tune`, and `RC10 opt` >> `Motor Interlock`, and save the changes.
3. Config/Tune >> Full Parameter >> set `AUTOTUNE_AGGR` : 0.05 (default 0.1),  `AUTOTUNE_AXES`: 0 (I suggest tuning roll, pitch, yaw separately)
4. fly in reality: take off in `ALT-HOLD` or `LOITER`(
recommend) flight mode, and push throttle to 50% ~ 59%, and activate your Radio Controller CH7 to HIGH, then PID Auto tuning will work. Wait UAV beep~ indicate that autotuning is done, and let your UAV land in Alt-Hold fight mode, and arm (You will see pid parameters saved). Finally, disabled your CH7 to LOW.

### UAV auto fight mode ([RTL](https://ardupilot.org/copter/docs/rtl-mode.html), [LAND](https://ardupilot.org/copter/docs/land-mode.html), [LOITER](https://ardupilot.org/copter/docs/loiter-mode.html), [AUTO](), [GUIDED](https://ardupilot.org/copter/docs/ac2_guidedmode.html), [ALT-HOLD](https://ardupilot.org/copter/docs/altholdmode.html#altholdmode-controls), [Brake](https://ardupilot.org/copter/docs/brake-mode.html))
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
#### 3. Other parameters :
* `RTL_ALT` : The minimum alt above home the vehicle will climb to before returning.  If the vehicle is flying higher than this value it will return at its current altitude.
* `WP_YAW_BEHAVIOR` : Determines how the autopilot controls the yaw during missions and RTL
* `MOT_SPIN_MIN` : Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range.  Should be higher than MOT_SPIN_ARM.

### Some Issue :
* [Configuring a Telemetry Radio using Mission Planner](https://ardupilot.org/copter/docs/common-configuring-a-telemetry-radio-using-mission-planner.html#common-configuring-a-telemetry-radio-using-mission-planner)
* [Telemetry Radio](https://ardupilot.org/copter/docs/common-sik-telemetry-radio.html#common-sik-telemetry-radio)
* [Setting Hover Throttle](https://ardupilot.org/copter/docs/ac_throttlemid.html#ac-throttlemid)
* [Non-GPS Navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html#common-non-gps-navigation-landing-page)

## How to be a drone maker : 
* [Flight Evaluation](https://flyeval.com/)
* [My ZD-850 Hexacopter Design](https://drive.google.com/file/d/19LNJ0DdcdDVKSDwWIAqSwMVJpiM6NzIc/view?usp=sharing)

## Drone with RTK :
### ***1. via radio telemery (serial port)***
#### Base (GCS mission planner) 
0. ***https://ardupilot.org/copter/docs/common-here-plus-gps.html***
1. Open Mission planner Initial Setup >> Optional Hardware >> RTK GPS Inject screen
2. Connect your F9P to computer via COM port and baud rate is 115200
3. Check the autoconfig, then set the surveyin accuracy in meter and time in second (you can use default setting: 2(m) and 60(s)) 
4. TIME Mode of base station requires: 5 GPS satellite signals +2 GLONASS satellite signals in the strength of 40 or more; or 5 GPS satellite signals +3 Beidou satellite signal in the strength of 40 or more.

#### Rover 
0. Notice that your ardupilot firmware version should higher than 3.6.9, i use 3.6.11 hexacopter.
1. Use I2C provide power (that is, plug the GPS module's GPS pin +5V and GND and insert to I2C).
2. Use your F9P pins insert to your pixhawk GPS pins.
3. Use mission planner connect to drone's via radio telemery.
4. Wait for rtk fixed.
5. (option) paramter list setting:   `EK2_ALT_SOURCE:2`, `EK2_POSNE_M_NSE : 0.1[meter] or 0.01[meter] (0.01 should use carefully)`

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
5. (option) paramter list setting: `EK2_ALT_SOURCE:2`, `EK2_POSNE_M_NSE : 0.1[meter] or 0.01[meter] (0.01 should use carefully)`

## How to train Darknet YOLOv4
* Yolo v4 paper:    [https://arxiv.org/abs/2004.10934](https://arxiv.org/abs/2004.10934)

* Yolo v4 source code:  [https://github.com/AlexeyAB/darknet](https://github.com/AlexeyAB/darknet)  
#### 0. Install cuda and cudnn
* [Install CUDA 10.0 and cuDNN v7.4.2 on Ubuntu 16.04](https://gist.github.com/matheustguimaraes/43e0b65aa534db4df2918f835b9b361d)
* [多版本CUDA与cuDNN管理](https://dinghow.site/2019/05/19/cuda-cudnn-version/)


#### 1. Compile on Linux
```
  git clone https://github.com/AlexeyAB/darknet.git
  cd darknet
  gedit Makefile
```
* GPU=1 to build with CUDA to accelerate by using GPU (CUDA should be in /usr/local/cuda)
* CUDNN=1 to build with cuDNN v5-v7 to accelerate training by using GPU (cuDNN should be in /usr/local/cudnn)
*  OPENCV=1 to build with OpenCV 4.x/3.x/2.4.x - allows to detect on video files and video streams from network cameras or web-cams.
*  LIBSO=1 to build a library darknet.so and binary runable file uselib that uses this library.
*  Choose your GPU capability (ARCH)
*  Notice that your cuda path (NVCC=/usr/local/cuda-10.0/bin/nvcc)
* Save and close the Makefile, and type `make` in terminal.
#### 2. Create my own dataset, and label
* [Download in Windows](https://tzutalin.github.io/labelImg/) choose Windows_v1.8.0 and unzip it.
* Create folder :
*  `git clone https://github.com/LiaoSteve/pascal-VOC.git`, and open it.   
*  
    ```  
    VOCdevkit
    |----VOC2007
        |----Annotations
        |----ImageSets    
        |    |---- Main            
        |----JPEGImages    
    voc_label.py
    generate_train.py
    ```
* Put your images to JPEGImages dir,and open labelimg dir, open data folder, edit predefined_classes.txt (type your class per line)
* Click labelImg.exe, choose `Open Dir` to `JPEGImages` images, and `Change Save Dir` to `Annotations` dir.
* Use `pascal VOC` label format, and start labeling your images.
* Now if your label work done, put `VOCdevkit` dir and `voc_label.py` into `darknet/data` dir.
* Open terminal and run create_imageSets.py, and check your ImageSets/Main dir, there are four .txt files:
    ```  
    cd VOCdevkit/VOC2007
    python create_imageSets.py    
    ```  
* Open voc_label.py, and revise the code line 7 (`classes = ["class1","class2"]`) to your classes :
    ```    
    cd ../../
    python voc_label.py
    ```
* Now in data dir, you will see `2007_train.txt`, `2007_val.txt`, and there are many .txt YOLO format labels in `VOCdevkit/VOC2007/labels` 
* (Optional) Copy .txt YOLO format labels to images dir :
  ```
  cp -r ./VOCdevkit/VOC2007/labels/*.txt ./VOCdevkit/VOC2007/JPEGImages/
  ```
* Create `obj.names` file in `darknet/data` dir, and type your classes name (each line one class) :
    ``` 
    gedit obj.names
    ```
* Create `obj.data` file :
    ``` 
    gedit obj.data
    ```
* Paste text (revise `classes`) below to `obj.data`:
  
    ```
    classes= 2
    train  = data/2007_train.txt
    valid  = data/2007_val.txt
    names = data/obj.names
    backup = backup/
    ```
#### 3. Download pre-trained weights [yolov4.conv.137](https://drive.google.com/file/d/1JKF-bdIklxOOVy-2Cr5qdvjgGpmGfcbp/view), and put it in `darknet` dir.

#### 4. Edit cfg file. 
* [AlexeyAB darknet README.md](https://github.com/AlexeyAB/darknet#how-to-train-to-detect-your-custom-objects)
- .cfg : filters=(classes + 5)x3

#### 5. Start training by using the command line: 
* In `darknet` dir :
```
./darknet detector train data/obj.data cfg/yolov4.cfg yolov4.conv.137 -map | tee -a train.log
```
* Or remote version with no GUI :
```
./darknet detector train data/obj.data cfg/my_yolov4.cfg yolov4.conv.137 -dont_show -mjpeg_port 8090 -map
```
#### 6. After training 
* Test image : 
```
./darknet detector test data/obj.data cfg/yolov4.cfg backup/yolov4.weight
```
* Test video and save :
```
./darknet detector demo data/obj.data cfg/yolov4.cfg backup/yolov4.weight video.mp4 -out_filename result.avi
```
* Test webcam :
```
./darknet detector demo data/obj.data cfg/yolov4.cfg backup/yolov4.weight -c 0
```
* Test mAP : 
```
./darknet detector map data/obj.data cfg/yolov4.cfg backup/yolov4.weight
```
* Test recall :
```
./darknet detector recall data/obj.data cfg/yolov4.cfg backup/yolov3.weight
```
#### 7. Python wrapper :
* https://github.com/LiaoSteve/Drone-GCS-and-AI/tree/django_app/darknet_yolo
#### 8. How to improve object detection : 
* https://github.com/AlexeyAB/darknet#How-to-improve-object-detection
  
### How to create custom dataset from OpenImage Dataset
* [https://github.com/theAIGuysCode/OIDv4_ToolKit](https://github.com/theAIGuysCode/OIDv4_ToolKit)
* Download images and labels :
  ```
  pip install -r requirement.txt
  python main.py downloader --classes Apple --type_csv train --limit 100  
  ```
* Revise `classes.txt`, and run python `convert_annotations.py`
  ```
  python convert_annotations.py
  ```
* Copy images to `VOC2007/JPEG` dir, and copy labels (yolo format .txt) to `labels` dir
* Copy `generate_train.py` to `darknet/data/` dir, run and train yolov4 :
  
  ```  
  python generate_train.py
  
  ./darknet detector train data/obj.data cfg/yolov4.cfg yolov4.conv.137 -map | tee -a train.log
  ```
### How to learn deep learning
* [https://www.youtube.com/watch?v=G_fsA-OUqNw&list=PL1w8k37X_6L9YSIvLqO29S9H0aZ1ncglu&index=1](https://www.youtube.com/watch?v=G_fsA-OUqNw&list=PL1w8k37X_6L9YSIvLqO29S9H0aZ1ncglu&index=1)

## Jetson AGX Xavier and NX
### Build darknet
1. nvcc
```
gedit ~/.bashrc
export CUDA_HOME=/usr/local/cuda
export PATH=$PATH:$CUDA_HOME/bin
export LD_LIBRARY_PATH=/usr/local/cuda/lib64
source ~/.bashrc
```
2. git clone darknet source code and revise parameters
```
git clone https://github.com/AlexeyAB/darknet.git
cd darknet
gedit Makefile
```
* GPU=1 
* CUDNN=1 
*  OPENCV=1 
*  LIBSO=1 
*  ARCH= -gencode arch=compute_72,code=[sm_72,compute_72]
* Save and close the Makefile, and type `make` in terminal.
* Download pre-trained weights [yolov4.conv.137](https://drive.google.com/file/d/1JKF-bdIklxOOVy-2Cr5qdvjgGpmGfcbp/view), [yolov4.weights](https://drive.google.com/file/d/1cewMfusmPjYWbrnuJRuKhPMwRe_b9PaT/view), and put it in `darknet` dir.
* Run darknet_video.py to test (connect your webcam)
### Fan mode
```
# choose one mode
sudo /usr/sbin/nvpmodel -d cool # use this
sudo /usr/sbin/nvpmodel -d quiet

# pwm 255
sudo jetson_clocks 

sudo jetson_clocks --store
sudo jetson_clocks 
sudo jetson_clocks --restore
```
### Install pip3
```
sudo apt-get install python3-pip
```
### Jetson system-monitor :
```
sudo pip3 install jetson_stats
sudo jtop
```
### VScode install from [JetsonHacks](https://www.jetsonhacks.com/2019/10/01/jetson-nano-visual-studio-code-python/) 
```
git clone https://github.com/JetsonHacksNano/installVSCode.git
cd installVSCode
./installVSCode.sh
code-oss
```
### dronekit
```
pip3 install pyserial
sudo pip3 install dronekit 
# permission on serial port
sudo adduser <your user name> dialout
sudo reboot
```

## License 
* Notice that our License is reserved

## Refference
1. YOLOv3
```
@article{yolov3,
  title={YOLOv3: An Incremental Improvement},
  author={Redmon, Joseph and Farhadi, Ali},
  journal = {arXiv},
  year={2018}
}
```
2. YOLOv4
```
@article{yolov4,
  title={YOLOv4: Optimal Speed and Accuracy of Object Detection},
  author={Alexey Bochkovskiy, Chien-Yao Wang, Hong-Yuan Mark Liao},
  journal = {arXiv},
  year={2020}
}
```




