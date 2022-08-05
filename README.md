# Extended Kalman Filter in C++

The fifth project in the Udacity Self-Driving Car Engineer Nanodegree: using an extended Kalman filter in C++ to estimate pedestrian pose and trajectory given radar and lidar readings.

## Description

The EKF code will use simulated output from a radar and lidar sensor to track a moving object. For this project, we will assume that the parsing of the raw sensor data has already been done successfully by another process, and this program will take information from the sensors regarding the location of the detected object.

![EKF Process Flowchart](https://user-images.githubusercontent.com/14826664/183125574-379ae12e-9438-473d-a05a-c0311f29209c.png)

Kalman filters help us combine a prediction and a measurement with an accurate estimation of the state of the object. Here is a one-dimensional illustration:

![Kalman Filter Diagram](https://user-images.githubusercontent.com/14826664/183124896-905e2ab1-f582-4895-9d81-381d0c14a312.png)

By applying these concepts to a 2D measurement space and using different prediction equations for each sensor, we can track an object as it moves around a plane.

## Requirements

- cmake >= 3.10
- make >= 4.1
- gcc/g++ >= 5.4
- uWebSockets
- [Eigen](https://eigen.tuxfamily.org/index.php)

To install uWebSockets, follow these instructions:
```
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
```

## Build instructions

1. Clone this repo
```
git clone https://github.com/ibvandersluis/udacity-CarND-P05-cpp-ekf.git
cd udacity-CarND-P05-cpp-ekf
```
2. Make build directory
```
mkdir build && cd build
```
3. Build with cmake
```
cmake ..
make
```

## Run

To run this project as intended, you will need the [Term 2 Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases/).

1. Run the EKF
```
./ekf
```
You should get back:
```
Listening to port 4567
```
2. Run the simulator and select 'Project 1/2: EKF and UKF'

When you do this, the EKF executable should report:
```
Connected!!!
```
3. Select either Dataset 1 or Dataset 2 and click start. You should see laser measurements appear in red and radar measurements appear in blue. The output of the EKF will appear as green triangle markers and should follow the car smoothly.

![EKF Simulator](https://user-images.githubusercontent.com/14826664/183123351-5cafaee7-8ee1-4e20-be41-b2cf7b462bf7.png)

> NOTE: If you want to run the EKF again, click the RESET button but restart the EKF executable before clicking START again!
