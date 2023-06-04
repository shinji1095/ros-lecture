# ros-lecture
This repogitory is for sharing the code of our group's final presentation. Rio and me were able to make these function
- color tracking
-  building server to communicate with LINE server
-  operate turtlebot3 using LINE message

Please check below to get more information of our project.
Thank you.

# Requirments
- Flask
- python-dotenv
- Gunicorn
- Opencv
- line-bot-api

# About Line Messaging API
LINE is a popular messaging and communication application that originated in Japan. They release SDK to making a chat bot and there are many information in the Internet. We use this app because it's easy for us to make our desire function above by modifing code in the Internet. And terchers are always use this application so they are familiar with it.  
If you want to get more information about LINE and LINE API, please check the URL below.  
https://developers.line.biz/en/services/messaging-api/

# How to
Please open a new terminal in each block.

1st  
turtle_mode  
roscore

2nd  
ssh -YC turtle@192.168.11.2  
roslaunch ros_lecture bringup.launch

3rd  
turtle_mode  
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/ros/map.yaml

4th  
turtle_mode  
source ~/ros_ws/competition_ws/devel/setup.bash  
rosrun competition_pkg main.py 
