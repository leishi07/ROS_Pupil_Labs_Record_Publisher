# ROS_Pupil_Labs_Record_Publisher
A ROS interface which reads the exported files of Pupil Labs eye tracking glasses. 
Pupil Labs eye tracking glasses (https://pupil-labs.com/) provides recording function which records the data and later offline analysis can be conducted. This repository reads the offline world image, normalized gaze and fixation and publishes them as ROS topics.

# Pre-request
Ubuntu 16.04 \
ROS Kinetic \
OpenCV 3.3.1

# Usage
From your Pupil Labs recording export directory, copy the following files into folder 'gaze_record/pupil_labs',

`world.mp4` \
`gaze_positions.csv` \
`fixation_positions.csv`

You can change the directory by modifying the path in ROS launch file `record_pub.launch`

To start the node:

`roslaunch record_pub record_pub.launch`

# ROS Topcis

**</pupil_capture/world>** \
Publishes the image of the world camera.

**</pupil_capture/gaze_std>** \
Publishes the normalized gaze positions. Message type is `geometry_msgs/PointStamped`

**</pupil_capture/fixation>** \
Publishes the fixation positions. Message type is `geometry_msgs/PointStamped`
