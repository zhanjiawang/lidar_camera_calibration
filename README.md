# lidar_camera_calibration
This is a simple laser radar camera external parameter calibration software that can calibrate by observing and adjusting parameters without using a calibration target. It is used for scenarios where external parameter requirements are not particularly strict or to provide a good initial external parameter value.

## Usage
#### 1. Requirement
```
1. ROS
2. PCL
```

#### 2. Build
```
cd ${YOUR_PROJECT_DIR}
catkin_make
```

#### 3. Run
```
### 1.Prepare keyframe point clouds and keyframe poses and place them in the data directory
### 2.Modify parameters in the config directory
### 3.run the program
source ./devel/setup.bash
roslaunch plane_localization start.launch
### 4.Obtain the results
---------------------------
transformation_matrix_vector size: 49892
cluster_transformation_matrix_vector size: 19614
cluster_transformation_time: 0.034689
---------------------------
fine_verify_octree_time: 0.084577
best_transformation_score: 1865 best_transformation_matrix: 
   -0.417447     0.908311    0.0266064   -0.0180196
   -0.908634    -0.417593 -9.00216e-05     0.061221
   0.0110289   -0.0242131     0.999646    -0.130588
           0            0            0            1
localization_time: 0.772197
```

#### 4. Example
```
The above figure achieves correct global localization (relocation) in an indoor scene of approximately 2500 square meters
```
