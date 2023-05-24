# Robocon Recruitments- Task 3.2

## Solution for Subtask A: Mastering PID Control

- A PID controller is a powerful tool that uses proportional, integral, and derivative terms to control a system. By mastering this tool, you'll be able to control the robot with precision and accuracy.
    - You can learn more about PID controllers from these resources: https://youtu.be/wkfEZmsQqiA and any other content you find for pid controllers. 

- To get started:
    - You'll need to use the Atom bot package again. (You may use another bot, but this guide shall follow the atom bot).
    - In this repository, there's a file called `pid.py`. This file needs to be placed in `Robotics_ws/src/atom/script`.
    - Once the file is in place, you need to run `chmod +x` on it to make it executable.
    - Then, in the original folder (`Robotics_ws`), you need to run `catkin_make` and `source devel/setup.bash`.
    - Finally, you can run `rosrun atom pid.py` for the script to start working. Make sure you first launch the world using the command from Task 1 (`roslaunch atom world.launch`).

- The `pid.py` script has some boilerplate code for implementing a PID controller. Currently, it asks you to enter an input velocity every 2 seconds. **You need to modify this file so that it asks for coordinates instead and the bot must reach those coordinates using PID control.** 

- To control the bot's movement, you can use the `odom` topic to get its current status and the `cmd_vel` topic to give it new velocities. Just be careful not to give it large velocities or it might behave unpredictably.
