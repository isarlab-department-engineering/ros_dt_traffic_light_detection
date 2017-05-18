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



Launch: "roslaunch traffic_light rospibot.launch" on the robot will launch:
- rospibot_network.py (from ros-joy-controller package)
- traffic_light_detection.py
- red_mask_detection.py
- grey_scale_detection.py
- talker.py (from ros-pi-camera-interface package)
- motor_hat_node (from motor_hat package)
