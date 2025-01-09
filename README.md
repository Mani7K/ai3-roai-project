# AI3 ROAI Project

## Prerequisites

- Docker-ROS https://github.com/TW-Robotics/Docker-ROS/tree/master

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
4. `roslaunch ai3-roai-project environment.launch`
    - This will start the environment
    - load gazebo world
        - Option A: gazebo_small_house\
        Complex map containing multiple rooms and additional furniture
        - Option B (default): simple\
       This is a small map with a few chairs, gazebo runs much smoother using this world\
       Ideal for demo purposes / testing
    - spaws the robot, brings up controllers, etc
5. `roslaunch ai3-roai-project object_mapping.launch`
   - This will start a node that explores and maps the world. it subscribes to the `/object_detection` topic and will place detected objects on the map

You should now see a running gazebo instance, showing either the small_house or maze world including a mir100 with a camera (small red box) and a laser scanner (small red cylinder) mounted.

The image gets published to the topic `/image_raw`. It can be visualized e.g. by using rviz.
The laser scan data gets published to the topic `/scan`. It can be visualized using rviz as well.
When launching the SLAM node a map is published to the topic `/map`, its visualized in the preconfigured rviz instance.

#### Object Detection

The object detection node publishes only if a chair or table is detected. The published message is an array of json object with the following structure:

```json
[
  {
    "class": "string",          // Detected object class
    "confidence": float,        // Confidence score (0.0 to 1.0)
    "bbox": [int, int, int, int] // Bounding box [x, y, width, height]
  },
]
  ...
```

RVIZ Laserscan visualization example:

1. Add => RobotModel
2. Global Options: Set `Fixed Frame` to `base_link`
3. Add => LaserScan => Set `Topic` in the `LaserScan`-Tab to `/scan`
