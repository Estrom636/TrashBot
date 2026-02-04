# **TrashBot**

This repository is for an autonomus robot.


## **Chassis**

The chassis of this robot is made of U-Channel in the shape of an H. Has two wheels in the middle on either side for drive and a four Omni wheels in the corners for balance.


## **Electronics**
### **Control**
All controls are programmed onto an ESP32 using Arduino IDE.

### **Motors**
Currently has two [435rpm Yellow Jacket Planetary Gear Motors](https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/) from Gobilda. The motor uses a [motor controller](https://www.gobilda.com/1x15a-motor-controller-9-14v-input/) controled by a pwm signal.

### **Lidar**
Currently a D500 from LDROBOT.

### **Camera**
_No camera yet_

### **Light**
Currently uses a [LED Headlight](https://www.gobilda.com/led-headlight-for-ftc-275-lumens-dimmable-pwm-controlled/) from Gobilda to indicate that the robot is armed.


## **Code**
All code is writen in Arduino IDE. 
### **Drive**
The basic drive code uses [Bluepad32](https://bluepad32.readthedocs.io/en/latest/) to use a controller for basic driving of the chassis. Has an arm and disarm trigger for saftey. If disarmed the robot will not move.\
The 2nd basic drive code is based on the 1st basic drive code but added on the lidar to disarm and slow the robot if it read stuff in range.\
The encoder test uses the encoders to drive the robot forward and turning.

### **Lidar**
The basic lidar code is to test that the lidar works by displaying the distance into the serial monitor. 

### **Camera**
_No camera yet_
