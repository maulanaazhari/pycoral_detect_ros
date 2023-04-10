## 0. Edge TPU in ROS

This ROS package provides basic support for the Google Edge TPU, for example the Coral dev board or the USB accelerator.
To convert YOLO model into edgetpu-compatible tflite, refer to this repo: https://github.com/hunglc007/tensorflow-yolov4-tflite.git

## 1. ROS Setup

If you are using ROS Melodic, you need to install python3 dependencies in ROS. To do that, run the following:

### 1.0 Change the python3 version into python3.8

### 1.1 Install ROS Melodic
```bash
sudo apt install ros-melodic-full
```

### 1.2 How to use Python3 in ROS Melodic
This is a one time setup to enable python3 in ros melodic

#### Install rospkg for python3
```bash
sudo apt install python3-pip python3-all-dev python3-rospkg
```

#### Reinstall ROS
```bash
sudo apt install ros-melodic-desktop-full --fix-missing
```

#### Add a shebang line
this line will let the bash know what interpreter to user (Python2 or Python3). add this line to every python3 file!
```python
#!/usr/bin/env python3
```


### 1.3 Install ROS Dependencies
```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update

pip3 install --upgrade pip
pip3 install opencv-python
sudo apt install ros-melodic-vision-msgs ros-melodic-ros-numpy python3-pycoral libedgetpu1-std
```

[optional] ```pip3 install imagezmq imutils```

### 1.4 (Optional) Video and Camera Streamer
If you want to stream video from a file or camera, you can use **video_stream_opencv**
#### Install the package
```bash
sudo apt-get install ros-melodic-video-stream-opencv
```
#### Run the launch file
to run the streamer, you can run the launch file **camera_stream.launch** or **video_stream.launch**. Launch file example:
```xml
<?xml version="1.0"?>
<launch>
   <!-- launch video stream -->
   <include file="$(find video_stream_opencv)/launch/camera.launch" >
        <!-- node name and ros graph name -->
        <arg name="camera_name" value="videofile" />
        <!-- full path to the video file -->
        <!-- wget http://techslides.com/demos/sample-videos/small.mp4 -O /tmp/small.mp4 -->
        <arg name="video_stream_provider" value="/home/usrg/Downloads/out.mp4" />
        <!-- set camera fps to (video files not affected) -->
        <!-- <arg name="set_camera_fps" value="30"/> -->
        <!-- set buffer queue size of frame capturing to -->
        <arg name="buffer_queue_size" value="1000" />
        <!-- throttling the querying of frames to -->
        <arg name="fps" value="0.25" />
        <!-- setting frame_id -->
        <arg name="frame_id" value="camera" />
        <!-- camera info loading, take care as it needs the "file:///" at the start , e.g.:
        "file:///$(find your_camera_package)/config/your_camera.yaml" -->
        <arg name="camera_info_url" value="" />
        <!-- flip the image horizontally (mirror it) -->
        <arg name="flip_horizontal" value="false" />
        <!-- flip the image vertically -->
        <arg name="flip_vertical" value="false" />
        <!-- enable looping playback -->
        <arg name="loop_videofile" value="true" />
        <!-- start frame of video -->
        <arg name="start_frame" default="0"/>
        <!-- stop frame of video, -1 means the end of video -->
        <arg name="stop_frame" default="-1"/>
        <!-- visualize on an image_view window the stream generated -->
        <arg name="visualize" value="false" />
   </include>
</launch>
```
Run the launch file by:
```bash
roslaunch pycoral video_stream.launch
```
or for camera:
```bash
roslaunch pycoral camera_stream.launch
```

## 2. Usage
To run this package, you can just type the following on your terminal
```bash
roslaunch pycoral detect.launch
```
You can set various parameters in the detection launch file, such as:
- **display** for displaying the result in a window
- **compressed** for using compressed image instead of raw
- **topic_name** image stream topic name

```xml
<launch>
  <arg name="display" default="false" />
  <arg name="compressed" default="false" />
  <arg name="topic_name" default="input" />

  <node pkg="pycoral" name="detector" type="detect_coral" output="screen">
    <param name="model_path" value="$(find pycoral)/models/efficientdet_lite1_384_ptq_edgetpu.tflite" />
    <param name="label_path" value="$(find pycoral)/models/coco_labels.txt" />
    <param name="compressed" value="$(arg compressed)" />
    <param name="topic_name" value="$(arg topic_name)"/>
    <param name="display" value="$(arg display)"/>
  </node>
</launch>
```

## 3. Results
The video results can be seen in here: https://drive.google.com/drive/folders/1-NOWcU_wW81T5LQAPTITuM8CmTb11Yw3?usp=sharing

| Model Name | Inference Time (i7-6700) | Inference Time (NUC) | mAP in COCO |
| ---------- | -------------- | -------------- | ----------- |
| SSD Mobilenet V2 | 10-12 ms | 10-11 ms | 25.6% |
| SSDLite MobileDet | 12-14 ms | 12-13 ms | 32.9% |
| EfficientDet Lite1 | 70-80 ms | 68-74 ms | 34.3% |
| EfficientDet Lite2 | 120-130 ms | 120-127 ms | 36.0% |
| EfficientDet Lite3 | 140-160 ms | 135-148 ms | 39.4% |

## 4. TODO
- Create separate packages for pycoral_det, det_msgs (including srv), and 3D localization
