# AI3 ROAI Project

## Prerequisites
* Docker-ROS https://github.com/TW-Robotics/Docker-ROS/tree/master

## Startup
This project represents a ros package.\
When using the docker container described in prerequisites, clone/copy this repository into the `Docker-ROS/catkin_ws/src` directory.

Your folder structure should look like this:
```
Docker-ROS/
└── catkin_ws/
    └── src/
        └── ai3-roai-project
```

Inside the directory `Docker-ROS`, there is a file to run the docker container called `run_docker_from_hub.[sh|bat]`, which will also connect to a shell inside the container.

To install all required dependencies, which are currently not available inside the container, run this file:\
`/home/fhtw_user/catkin_ws/src/fhtw/ai3-roai-project/scripts/dependencies.sh`

After that, we can start the typical ros workflow:
1. Navigate to `/home/fhtw_user/catkin_ws/`
2. Run `catkin_make`
3. `source devel/setup.bash`
4. `roslaunch ai3-roai-project gazebo_small_house.launch`

You should now see a running gazebo instance, showing the small_house world including a mir100 with a camera (small red box) mounted.

The image gets published to the topic `/image_raw`. It can be visualized e.g. by using rviz.
