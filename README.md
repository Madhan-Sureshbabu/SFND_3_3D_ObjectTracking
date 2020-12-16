# Udacity Nanodegree : Sensor Fusion

## 3D Object Tracking 

The project aims to achieve the following :  
```
1. Comparison of different keypoint detection and description mathods
2. Object detection using YOLO V3 framework
3. Bounding box matching and tracking based on keypoint matches
4. Projecting Lidar data on image and comparing Time-To-Collision estimates from the two sensors.
```

The complete project pipeline can be seen below. 
<img src="docs/course_code_structure.png" width="700" height="400" />


### Basic Build Instructions 
1. Clone the repository.
2. Create build folder and compile 
```
$ mkdir build && cd build
$ cmake .. && make
```
3. Run executable
```
$ ./<executable_name>
```

### Project Rubric

#### FP.1 Match 3D Objects

Implemented the method "matchBoundingBoxes", which takes as input both the previous and the current data frame objects and provides as output the ids of the matched regions of interest (the boxID). Matches are the roi that have the highest number of keypoint correspondences.
  
#### FP.2 Compute Lidar-based TTC
Compute the time-to-collision in seconds for all 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.
   > The constant velocity motion model to find the time to collision. From the Lidar, we get several distance values and we can consider the average value for the distance. 

#### FP.3 Associate Keypoint Correspondences with Bounding Boxes
The bounding boxes that contain the maximum number of keypoint matches are matched together.
   > We use a vector of keypoint matches between the current and previous frame. There will be an error in the keypoint matches and to account that error and mitigate it to some extent, the keypoint matches were iterated and the mean distance value and the standard deviation was calculated. To remove the outliers the readings which were within one-fourth standard deviation from the mean distance value were filtered.

#### FP.4 Compute Camera-based TTC
Compute the time-to-collision in seconds for all matched 3D objects using only keypoint correspondences of points within the matched bounding box.
   > The TTC can be computed using the distance ratio between any 2 keypoint correspondences. So based on the Keypoint matches, the distance ratio was calculated for all key-points in the current and previous frame. The median value of the distance ratio was used to estimate the Camera-based TTC to mitigate outliers.

#### FP.5 Performance Evaluation 1
The logged TTC estimates below show that the Lidar TTC is way off in a couple of instances. The mean x-coordinate has been used as the distance between the ego-vehicle and the preceding vehicle. 

   > Following are the results with the ```FAST + BRIEF``` detector descriptors. 

|Frame     |0      |1      |2      |3      |4      |5      |6      |7      |8      |9      |
|----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|Camera TTC|16.2688|16.7673|17.7801|15.3782|16.118 |16.3351|16.4664|17.6808|17.3522|18.6324|
|Lidar TTC |12.5156|12.6142|14.091 |16.6894|15.7465|12.7835|11.9844|13.1241|13.0241|11.1746|


#### FP.6 Performance Evaluation 2
1. Several combinations of detectors and descriptors have been tested for estimating the Camera TTC. Based on the data it can be inferred that the ORB detector performs the worst. 

Also it can be seen that the combination of FAST detectors (threshold = 100) and + ORB descriptors produced a stable output without large variations in Camera TTC.


[Spreadsheet link](./docs/FP6.csv)

|Detector_type|Descriptor_type|ImgNumber|TTC_Lidar|TTC_Camera|
|-------------|---------------|---------|---------|----------|
|FAST         | BRISK         |0        |         |          |
|FAST         | BRISK         |1        |12.2891  |12.551    |
|FAST         | BRISK         |2        |13.3547  |12.8653   |
|FAST         | BRISK         |3        |16.3845  |12.6318   |
|             |               |         |         |          |
|FAST         | BRIEF         |0        |         |          |
|FAST         | BRIEF         |1        |12.2891  |11.6697   |
|FAST         | BRIEF         |2        |13.3547  |12.0207   |
|FAST         | BRIEF         |3        |16.3845  |13.7125   |
|             |               |         |         |          |
|FAST         | ORB           |0        |         |          |
|FAST         | ORB           |1        |12.2891  |12.5425   |
|FAST         | ORB           |2        |13.3547  |12.5265   |
|FAST         | ORB           |3        |16.3845  |13.7667   |
|             |               |         |         |          |
|FAST         | FREAK         |0        |         |          |
|FAST         | FREAK         |1        |12.2891  |12.9991   |
|FAST         | FREAK         |2        |13.3547  |12.093    |
|FAST         | FREAK         |3        |16.3845  |14.6114   |
|             |               |         |         |          |
|ORB          | BRISK         |0        |         |          |
|ORB          | BRISK         |1        |12.2891  |10.676    |
|ORB          | BRISK         |2        |13.3547  |13.5068   |
|ORB          | BRISK         |3        |16.3845  |11.2793   |
|             |               |         |         |          |
|ORB          | BRIEF         |0        |         |          |
|ORB          | BRIEF         |1        |12.2891  |12.2865   |
|ORB          | BRIEF         |2        |13.3547  |14.042    |
|ORB          | BRIEF         |3        |16.3845  |19.9017   |
|             |               |         |         |          |
|ORB          | ORB           |0        |         |          |
|ORB          | ORB           |1        |12.2891  |19.0873   |
|ORB          | ORB           |2        |13.3547  | -inf     |
|ORB          | ORB           |3        |16.3845  |21.8644   |
|             |               |         |         |          |
|ORB          | FREAK         |0        |         |          |
|ORB          | FREAK         |1        |12.2891  |12.2074   |
|ORB          | FREAK         |2        |13.3547  |11.1048   |
|ORB          | FREAK         |3        |16.3845  |11.3288   |
|             |               |         |         |          |
|AKAZE        | BRISK         |0        |         |          |
|AKAZE        | BRISK         |1        |12.2891  |11.8455   |
|AKAZE        | BRISK         |2        |13.3547  |13.1113   |
|AKAZE        | BRISK         |3        |16.3845  |13.0918   |
|             |               |         |         |          |
|AKAZE        | BRIEF         |0        |         |          |
|AKAZE        | BRIEF         |1        |12.2891  |12.101    |
|AKAZE        | BRIEF         |2        |13.3547  |13.519    |
|AKAZE        | BRIEF         |3        |16.3845  |12.0061   |
|             |               |         |         |          |
|AKAZE        | ORB           |0        |         |          |
|AKAZE        | ORB           |1        |12.2891  |11.6102   |
|AKAZE        | ORB           |2        |13.3547  |13.5195   |
|AKAZE        | ORB           |3        |16.3845  |12.745    |
|             |               |         |         |          |
|AKAZE        | FREAK         |0        |         |          |
|AKAZE        | FREAK         |1        |12.2891  |11.5026   |
|AKAZE        | FREAK         |2        |13.3547  |12.7074   |
|AKAZE        | FREAK         |3        |16.3845  |12.827    |
|             |               |         |         |          |
|SHITOMASI    | BRISK         |0        |         |          |
|SHITOMASI    | BRISK         |1        |12.2891  |13.3689   |
|SHITOMASI    | BRISK         |2        |13.3547  |13.0305   |
|SHITOMASI    | BRISK         |3        |16.3845  |12.3202   |
|             |               |         |         |          |
|SHITOMASI    | BRIEF         |0        |         |          |
|SHITOMASI    | BRIEF         |1        |12.2891  |13.7746   |
|SHITOMASI    | BRIEF         |2        |13.3547  |13.308    |
|SHITOMASI    | BRIEF         |3        |16.3845  |11.3769   |
|             |               |         |         |          |
|SHITOMASI    | ORB           |0        |         |          |
|SHITOMASI    | ORB           |1        |12.2891  |13.603    |
|SHITOMASI    | ORB           |2        |13.3547  |13.2121   |
|SHITOMASI    | ORB           |3        |16.3845  |12.884    |
|             |               |         |         |          |
|SHITOMASI    | FREAK         |0        |         |          |
|SHITOMASI    | FREAK         |1        |12.2891  |13.7616   |
|SHITOMASI    | FREAK         |2        |13.3547  |11.8519   |
|SHITOMASI    | FREAK         |3        |16.3845  |11.4126   |
|             |               |         |         |          |
|BRISK        | BRISK         |0        |         |          |
|BRISK        | BRISK         |1        |12.2891  |12.8659   |
|BRISK        | BRISK         |2        |13.3547  |16.5957   |
|BRISK        | BRISK         |3        |16.3845  |12.7803   |
|             |               |         |         |          |
|BRISK        | BRIEF         |0        |         |          |
|BRISK        | BRIEF         |1        |12.2891  |10.1889   |
|BRISK        | BRIEF         |2        |13.3547  |16.1327   |
|BRISK        | BRIEF         |3        |16.3845  |11.5662   |
|             |               |         |         |          |
|BRISK        | ORB           |0        |         |          |
|BRISK        | ORB           |1        |12.2891  |11.7327   |
|BRISK        | ORB           |2        |13.3547  |15.829    |
|BRISK        | ORB           |3        |16.3845  |14.2885   |
|             |               |         |         |          |
|BRISK        | FREAK         |0        |         |          |
|BRISK        | FREAK         |1        |12.2891  |11.2939   |
|BRISK        | FREAK         |2        |13.3547  |19.6872   |
|BRISK        | FREAK         |3        |16.3845  |12.2048   |



