# nathan_follow_path_pckg


## Organization
All scripts are located in "scripts" folder. Simulations meant to be run within Gazebo or on the real turtlebot hardware is in the "GAZEBO_ROS_sims" folder and the "scripts_sims" folder contains all the python script simulations that run in isolation. 

## Gzebo/Hardware sims

First clone the repo to the src folder of your ROS2 workspace

```
cd existing_repo
git remote add origin http://10.251.72.180/avl-summer-24/nathan_follow_path_pckg.git
git branch -M main
git push -uf origin main
cd .. 
colcon build
source install/setup.bash
```

## Gazebo simulations instructions 
Install Gazebo from https://gazebosim.org/home. 

open a terminal and run 
'''
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
'''
To run teleop control open a new terminal and run
'''
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
'''

To launch the sims in individual terminals run the following commands to run the noise indicator node, topic switch node, data collection node, and controller node
'''
python3 boolean_pub.py
python3 gazebo_odom_to_pose.py
python3 data_caollector.py
python3 tbot_EKF_node.py
'''
then run plotter.py in your editor of choice to analyze the trajectory. 

## Turtlebot Hardware instructions
If you are NOT running the sim in gazebo then adjust the tbot_EKF_node.py script to ensure it publishes the right topic for velocity commands for the specific turtlebot and subscribes to the correct pose topic from MOCAP. Then run these scripts in individual terminals. Keep in mind that the actuation and measurement rates (dt_act and dt_meas) may need to be adjusted in tbot_EKF_node.py based on your MOCAP and Gazebo installation. 
'''
python3 boolean_pub.py
python3 data_caollector.py
python3 tbot_EKF_node.py
'''
then run plotter.py in your editor of choice to analyze the trajectory. 

# Boolean publishing node
The node created in boolean_pub.py publishes a random boolean variable evey 5 seconds. The node created in tbot_EKF_node.py subscribes to this topic to determine which estimator will be used (good or jumpy). The boolean publishing node in  boolean_pub.py does not need to be active for the tbot_EKF_node to run. 

# Turtlebot estimator and controller node. 
The controller and estimator parameters are defined within the initializzation of the tbto_EKF_node class. These include but are not limited to measurement rate, control input rate, jumpy gains, well behaved gains, threshhold values to switch controllers and linear speed regulation parameters. 

## Script simulations
The script simulations can all be ran in your preferred editor as long as numpy and matplotlib are properly installed. Each script only considers the jumpy, bias or well behaved situations in isolation. 

