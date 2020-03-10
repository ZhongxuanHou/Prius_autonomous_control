# Lab Session 4

This documentation describes the use of ROS, openCV and Point Cloud library in order to autonomously drive a simulation of Toyota Prius while avoiding obstacles and people in its path with the help of it's sensor data.
#
#```
#source /opt/ros/kinetic/setup.sh
#cd ~
#mkdir -p catkin_ws/src
#cd catkin_ws/src
#catkin_init_workspace
#git clone git@gitlab.3me.tudelft.nl:cor/me41025/students-1718/lab4/group07.git
#git clone git@gitlab.3me.tudelft.nl:cor/me41025/simulator.git
#cd ..
#catkin_make
#source devel/setup.sh
#```
The simulation can be run using the following command:
```
roslaunch control_solution solution.launch
```
The simulation makes use of 3 packages, each of which are briefly explained as follows:

## opencv_solution
This package contains the node named **opencv_solution_node** that subscribes to the topic **/prius/fron_camera/image_raw** and publishes to the topics **/opencv_solution_node/visual** and **opencv_solution_node/detections**. This node makes use of the openCV library to detect the presence of people in the input image. The node uses the **HOGdetector** from the library to detect people in **front_camera** images. It makes use of **defaultPeopleDetector** using **getDefaultPeopleDetector()** function. The node then uses the **cv::rectangle** function to draw green boxes around the detected person. The resulting images with green boxes are published to the topic **/opencv_solution_node/visual**. The **cv::Rect** detections are converted to **vision_msgs/Detection2DArray** and published to the topic **opencv_solution_node/detections**.

## pcl_solution
This package contains the node named **pcl_solution_node** that subscribes to the topic **/point_cloud** of type **sensor_msgs/PointCloud2** and publishes it's results to the topic **pcl_solution_node/detections** of type **vision_msgs/Detection3DArray**. This node makes use of the point cloud library to detect obstacles, especially barrels. Firstly, the node uses this library to segment the plane from the input data and only keeps the points that are not a part of this plane. The node then uses the euclidean cluster extraction from pcl to extract the point cloud clusters from the filtered data. These filtered clusters are then converted into 3D bounding boxes by extracting the centers and extremums of the clusters, which are then published.  

## control_solution
This package contains the node named **control_solution_node** that subscribes to both **opencv_solution_node/detections** and  **pcl_solution_node/detections** and publishes to the topic **/prius** of type **prius_msgs/Control**. This node makes use of a class named **Controller** which implements the following control algorithm:
```
Filter the 3D obstacle detections and keep those with the center in front of the car with x > 0  and within the radius of 6 meters.

If the updated detections set is empty, then drive forward. i.e.
throttle = 1
steer = 0

if the detection set is not empty, get the closest object

if the y of the center of the closest object is > 0, steer right and drive. i.e.
steer = -1
throttle = 1

else, steer left and drive. i.e.
steer = 1
throttle = 1

if the person is detected and the area of detection is larger than 25000 pixels,
brake the car and never drive again
```
The class, Controller makes use of the following functions to control the vehicle:
1. **CalculateRadius()** : calculates the distance of the obstacle to the car
2. **controlVehicle()** : implements the control algorithm described above
3. **openCVcallback()** : filters out the unwanted opencv detections and calls the controlVehicle() function
4. **PCLcallback()** : filters out the unwanted obstacle detections and calls the controlVehicle() function  
