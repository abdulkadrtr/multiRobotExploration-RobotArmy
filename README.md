# Multi Robot Exploration
![pic](https://user-images.githubusercontent.com/87595266/232087653-15e75801-908e-4017-930c-543008c47192.png)

The Autonomous Exploration Army project is a cutting-edge initiative that aims to create a team of robotic vehicles for autonomous exploration. The team consists of 1 aerial vehicle, or drone, and 2 ground vehicles, which work collaboratively to map and explore unknown areas. Equipped with lidar sensors, the ground vehicles are responsible for mapping the terrain, while the aerial vehicle provides aerial imaging support to monitor the ground vehicles from above.

The heart of the system is the Autonomous Exploration Control Center, which centrally manages all the robots. It sends messages to the ground vehicles, providing them with optimal routes for exploration based on the combined maps obtained from the ground vehicles. Additionally, it receives camera images from the aerial vehicle, allowing it to have a comprehensive view of the entire area.

Using this coordinated approach, the Autonomous Exploration Control Center commands the aerial vehicle to position itself in a strategic location where it can have a clear view of both ground vehicles. This enables the team to effectively explore and map the entire area. The merged maps from the ground vehicles and the aerial imagery provide a consolidated map of the environment, allowing for further analysis and decision-making.

The utilization of ROS 2 and Gazebo simulation environment in this project has facilitated realistic simulations for testing and development of the autonomous exploration army. This cutting-edge project showcases the power of autonomous robotics in exploring unknown terrains and gathering valuable information, with potential applications in fields such as search and rescue, environmental monitoring, and more.

In conclusion, the Autonomous Exploration Army project demonstrates the capabilities of a team of autonomous robotic vehicles working together to explore and map unknown areas. With the use of advanced technologies such as lidar sensors, aerial imaging, and centralized control, this project has the potential to revolutionize the field of exploration and create new opportunities for autonomous robotics.

The project utilized the TurtleBot3 model and the SJTU Drone model for implementation.

## How Does It Work


To run the project, you will need to have ROS2 and Gazebo simulation installed on your system. Follow the instructions below:

1 - Install ROS2 and Gazebo simulation on your system.

2 - Create a ROS2 workspace (e.g., `ros2_ws`) and inside the workspace, create a src directory.

3 - Clone the project repository to the src directory using the following command:

`git clone https://github.com/abdulkadrtr/multiRobotExploration.git`

4 - Build the project using colcon build command in the ros2_ws directory:

`cd ros2_ws`

`colcon build`

5 - Source the project using the following command: `source install/setup.bash`

6 - Launch the Gazebo environment and robots using the following command in a new terminal:

`ros2 launch turtlebot3_gazebo multi_robot_launch.py`

7 - Launch the map merging package using the following command in another terminal:

`ros2 launch merge_map merge_map_launch.py`

This will start the map merging process and display the merged map in an RViz2 window, along with the real-time paths of the robots and the live image feed from the drone.

8 - Finally, in a third terminal, run the following command to start the autonomous exploration center:

`ros2 run multi_robot_exploration control`

With these steps, your project is now up and running, and the robots will start autonomously exploring the environment and merging maps to create a single map for visualization.

## Demo 

You can watch the video below for demo and detailed project presentation.

https://youtu.be/6FtEvvi4lk4

## A scene from the robots work

![Screenshot from 2023-04-14 15-09-45](https://user-images.githubusercontent.com/87595266/232044431-143e2592-d4f9-404b-89fd-243b9af53d68.png)



![rosgraph](https://user-images.githubusercontent.com/87595266/232061251-64c3ed55-8297-4057-86f8-11599ae4cfa8.svg)


![rosgraph](https://user-images.githubusercontent.com/87595266/232061592-6647db9b-791d-439f-be2a-52df373e54c1.png)
