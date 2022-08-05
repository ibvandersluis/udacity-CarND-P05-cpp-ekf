# udacity-CarND-P05-cpp-ekf
The fifth project in the Udacity Self-Driving Car Engineer Nanodegree: using an extended Kalman filter in C++ to estimate pedestrian pose and trajectory given radar and lidar readings.

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
4. Run
```
./ekf
```
