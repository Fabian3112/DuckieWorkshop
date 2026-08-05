# Workshop Idea

The children come for one day for about four hours. In the first two hours they can learn a little bit of programming in Python using the drawing turtle. Then they can program on the duckie bot. 

## Setup

To avoid complicated Setups on the Laptops of the students the thought is to enable Programming directly on the Duckiebots. For this I need a Docker container, running a jupiter notebook on the duckie bots, that can be accessed using a browser. This means all code will have to run on the Duckiebots itself. 

The robot uses ROS and will have to communicate with other nodes running in different docker containers on the robot using ROS. This should still work with poor Wi-Fi connection or many Robots simultaneously. The children should not have to work with r to keep it simple. Instead, a new Robot class will be created. The Students can work on that class and the class like accessing the camera image or setting the velocity, while the class performs the ROS communication.

The Robot class needs to be setup in a way that allows different tasks for children of different age groups. For example Computer Vision for lane detection for older children and Intersection handling with commands like straight(0.1), or turn(90) for younger children. 

Other Tasks could be:
    - control engineering for lane following
    - AI for rubber duck detection and Duckiebots
    - Stop if obstacle in front (using TOF sensor)
    - ...

Ideally the pupils only have to replace a function in the robot class. Without having to work in the class directly but rather in the Jupiter notebook. 

## Webpage
Additionally the Robot should provide a Web-frontend with which People can interact without having to Program. The Web-frontend probably has to communicate with the nodes using ros. This should allow the web-frontend to receive images and display them, like an image showing where white, and yellow lines where detected, where the robot plans to drive or where the robot detects obstacles or other robots. It should also be possible to start and stop the robot on the frontend. When the robot has stopped, it should take as little computation as possible.

## Rosbridge server
The duckiebots already run a RIS bridge container, exposing rostopics on web endpoints, that I might be able to use in the Web-Frontend


# Structure and detailed setup

## Implementation Start: Docker + WebFrontend

### Container runtime assumptions
- ROS distro: ROS Noetic
- Ubuntu version: 20.04 (Focal)
- ROS master location: `http://{robot_name}.local:11311`
- Bridge path for web frontend uses rosbridge-websocket

### Docker image behavior
- Base image is `ros:noetic-ros-base-focal`.
- Container starts two services:
    - Jupyter Lab on port `8888`
    - Static WebFrontend on port `8080`
- Startup script resolves robot identity from `ROBOT_NAME` (or hostname fallback) and exports ROS environment variables:
    - `ROS_MASTER_URI`
    - `ROS_IP`
    - `ROS_HOSTNAME`

### Recommended run command on robot
Use host networking so `.local` name resolution and ROS topic discovery are more reliable:

```bash
docker build -t duckie-workshop:dev -f dockerfile .
docker run --rm -it \
    --network host \
    -e ROBOT_NAME=<robot_name> \
    duckie-workshop:dev
```

### Access points
- Notebook: `http://{robot_name}.local:8888/lab`
- WebFrontend: `http://{robot_name}.local:8000`
