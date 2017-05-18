# ros-traffic-light-detection
ROS package for traffic light signals detection



ROS Nodes: 

- red_mask_detection.py:
runs on the robot, subscribing to image_topic and scanning the image looking for semaphore's red colour components

- grey_scale_detection.py:
runs on the robot, subscribing to image_topic and scanning the image looking for semaphore's light peaks

- traffic_light_detection.py:
runs on the robot, putting together the informations from the two previous Nodes and publishing the result on its topic



Arduino Code: 

- legoSemaphore.ino:
controls a Arduino Nano semaphore, allowing to light up the LED and swap between red and green



Launch:
- use "roslaunch traffic_light rospibot.launch" to launch all the Nodes needed for the traffic light detection
