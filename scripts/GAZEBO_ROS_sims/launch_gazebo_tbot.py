import subprocess
import time

# Function to run a command
def run_command(command):
    subprocess.Popen(command, shell=True, executable='/bin/bash')

# Set the TurtleBot3 model
turtlebot_model = 'burger'  # Change to waffle or waffle_pi as needed
export_command = f'export TURTLEBOT3_MODEL={turtlebot_model} && '

# Launch TurtleBot in an empty Gazebo world
gazebo_command = export_command + 'ros2 launch turtlebot3_gazebo empty_world.launch.py'
run_command(f'gnome-terminal -- bash -c "{gazebo_command}; exec bash"')

# Wait a few seconds for Gazebo to launch
time.sleep(5)

# Launch the teleoperation node
teleop_command = export_command + 'ros2 run turtlebot3_teleop teleop_keyboard'
run_command(f'gnome-terminal -- bash -c "{teleop_command}; exec bash"')
