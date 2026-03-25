# lidar_camera_calibration
This is a simple laser radar camera external parameter calibration software that can calibrate by observing and adjusting parameters without using a calibration target. It is used for scenarios where external parameter requirements are not particularly strict or to provide a good initial external parameter value.

## Usage
#### 1. Requirement
```
1. ROS
2. PCL
3. OpenCV
```

#### 2. Build
```
cd ${YOUR_PROJECT_DIR}
catkin_make
```

#### 3. Run
```
### 1.Prepare rosbag with image message and lidar point cloud message
### 2.Modify parameters in the config directory
### 3.run the program
source ./devel/setup.bash
roslaunch lidar_camera_calibration start.launch
### 4.adjust external parameters and observe results

按q增加roll  按a减少roll
按w增加pitch 按s减少pitch
按e增加yaw   按d减少yaw
按r增加x     按f减少x
按t增加y     按g减少y
按y增加z     按h减少z
非阻塞键盘读取演示 (按空格退出)
数据准备完成，可以调整标定参数!
减少z
0 0 0 0 0 -0.05
    1     0     0     0
    0     1     0     0
    0     0     1 -0.05
    0     0     0     1
减少z
0 0 0 0 0 -0.1
   1    0    0    0
   0    1    0    0
   0    0    1 -0.1
   0    0    0    1
程序结束
```

#### 4. Example
```
The above figure shows the effect of extrinsic calibration.
```
<img width="2485" height="1412" alt="image" src="https://github.com/user-attachments/assets/9ab90d26-0925-4b78-9a7d-9025556c6c53" />
