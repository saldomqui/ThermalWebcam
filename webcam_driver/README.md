# flir_module_driver

Driver for Lepton flir 3.5 thermal cameras

# compile flir_thermal_module
- cd rasberry_ws
- rosdep install --from-paths src --ignore-src -r -y
- catkin_make --only-pkg-with-deps flir_module_driver

# UVC compilation issues
If during compilation you have issues with UVC library dependencies follow these steps:
- cd thirdparty/libuvc
- mkdir build
- cd build
- cmake ..
- make
- sudo make install

this should compile and install libuvc properly

# set permission to use the device as normal user
- roscd flir_thermal_module
- sudo cp cfg/99-flir-thermal-camera.rules /etc/udev/rules.d/

# flir_module_driver parameters
  - **min_range_meas** minimum measurement range for scaling to pixel value [0,255]
  - **max_range_meas** maximum measurement range for scaling to pixel value [0,255]
  - **ir_img_color** set to true to publish ir temp-coded RGB color image, false for grayscale. Every pixel will be scaled on [0,255] following the range above
  - **auto_gain** set to true to autoadjust the scale instead of using a fix measurement range
  - **serial_number** on a console type dmesg | grep SerialNumber. Check the serial number of the thermal camera and copy it here
- note that you can modify the launch file to launch several camera node drivers by providing the corresponding serial number for each camera 
  - **capture_time** this is the time delay [seconds] since the image was captured till it is received. For these cameras it has been estimated to be around 180 ms
  - **publish_raw_data**  whether to publish or not raw sensor data in __std_msgs::UInt16MultiArray__ format. This data has not been treated and it has been read from the sensor 
  
# launch file arguments
  - **debug** wether to show or not in a rqt_image_view window the thermal image captured
  - **ir_img_color** wether to publish the termal image in RGB color-coded temperature or in single channel grayscale. 
  - **capture_time** the time delay of capture of the camera that is passed as local parameter to the node
  - **publish_raw_data**  whether to publish or not raw sensor data in __std_msgs::UInt16MultiArray__ format. This data has not been treated and it has been read from the sensor 
  - **serial_number** serial number of the thermal camera 
  
# launch flir_thermal_module

__roslaunch flir_module_driver flir_thermal_module.launch__ <args>

the node will publish : 
- **/flir_module_driver/thermal/image_raw** Thermal image (RGB or grayscale)
- **/flir_module_driver/thermal/temp_meas_range** ROS topic containing min-max scale range values used for scaling pixels from raw measurements to px values, and instantaneous min-max measurement values of current frame
- **/flir_module_driver/thermal/raw_data** raw sensor data in __std_msgs::UInt16MultiArray__ format. It is published only if publish_raw_data == true

if you call service **/flir_module_driver/thermal/fit_range** of type _std_msgs/Trigger_ the min-max values of the scaling range will be set to current min-max values of the first frame that arrives after calling the service  

![Thermal image captured with Lepton flir 3.5 camera color coded temp](https://github.com/saldomqui/README_linked_documents/blob/main/flir_module_driver/screenhot_thermal_image_color.png)
![Thermal image captured with Lepton flir 3.5 camera gray scale coded temp](https://github.com/saldomqui/README_linked_documents/blob/main/flir_module_driver/screenhot_thermal_image_gray.png)


