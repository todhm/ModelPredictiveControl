# CarND-Model Predictive Control. 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[image1]: ./pic/mpc_pic.png
[image2]: ./pic/equation.png

This is part of udacity self-driving car nanodegree project part5. In this project I have runed the car in the simulator using vehicle model. 
* In this model 6 state variables and 2 actuator variables had been used.
* Among 6 state varaibles 4 varaible represent state of vehicle **x**, **y** , **psi** (orientation angle of vehicle),  
  **v** (velocity of vehicle) and 2 error varaible from trajectory **cte**(cross track error),**epsi**(orientation error ).
* 2 actuator varaible **delta**(steering_angel) and **a**(accelration) are the varaible which control vehicle. 
* Model construct vehicle trajectory using 3 order polynomialmodel and adjusting car along trajectory. 

The source code of this project consists with followings.
* main.cpp: Recieve data and process data from simulator.
* MPC.h/MPC.cpp: code to construct model. 

 

The Simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

---

## Basic Build Instructions

I used xcode as main IDE. You can execute project with ide_profiles/xcode/MPC.xcodeproj file. 

Also you can execute project with following steps.
0. Install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems and [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for windows.
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./mpc`  
5. Run the simulator and watch the results.

---
## Model Construction Steps.

### Set Cost for model. 
* Following value have been used to set cost that is used to adjust state varaible. 
    * (1) Error from trajectory -> **cte** and **epsi**. 
    * (2) Deviation from reference velocity. 
    * (3) Actuator varaibles -> **a**, **delta** (Reducing unnecessary acceleration and wheel steering during drive) 
    * (4) Difference of actuator variables between timestamp. **d_a**, **d_delta**

### Update State. 
* Model have been updated with following equation. 

![alt text][image2]

### Predict Trajectory of car. 

* Using Vehicle model and state from simulator construct trajectory fitting 3 order polynomial. 
    
    


---

## Results
* The Car run with this MPC

[![alt text][image1]](https://www.youtube.com/watch?v=kbLuME4ARJY&feature=youtu.be)

---
## Discussion  
#### The crucial tuning point that I have struggled to finish this project.
* Converting map's coordinate to car's point of view were important since we build trajectory from car. 
