# Setup

## ssh
connect to the robot using ssh :
ssh duckie@<robot_name>
password: quackquack

## init docker (Only done once)

Copy form git:
``` git clone ... ```

Go into folder
``` cd ./Duckieworkshop ```

Build Docker (requires Internet Connection and will download for some time). `catkin_make` for the Duckietown messages runs here, once, not on every container start.
```docker build -t duckieworkshop:latest . ```

Create the container so it starts automatically whenever the robot boots (and restarts if it crashes):
```docker run -d --restart unless-stopped --network=host -e ROBOT_NAME=<ROBOTNAME> --name duckieworkshop duckieworkshop:latest```

Jupyter is then at http://<ROBOTNAME>.local:8888 and the frontend at http://<ROBOTNAME>.local:8765.

Open a shell inside the running container (ROS and duckietown_msgs are already sourced)
```docker exec -it duckieworkshop bash```

Show the startup log
```docker logs -f duckieworkshop```

Stop it (it stays stopped, also after a reboot, until you start it again)
```docker stop duckieworkshop```
```docker start duckieworkshop```

After changing code: rebuild the image and recreate the container
```
docker build -t duckieworkshop:latest .
docker rm -f duckieworkshop
docker run -d --restart unless-stopped --network=host -e ROBOT_NAME=<ROBOTNAME> --name duckieworkshop duckieworkshop:latest
```

A container created earlier with `docker run -it` can get autostart without recreating it:
```docker update --restart unless-stopped duckieworkshop```
It still runs the old image (with catkin_make on every start) until you rebuild and recreate it as above.

If testing (including ros master in the docker don't do it on the actual duckiebot)
```docker run -d --network host -e ROS_MASTER_URI=http://tick.local:11311 --name duckieworkshop duckieworkshop:latest```

Copying docker image from robot to robot (Only a quick fix) (all these commands take a long time)
```docker export duckieworkshop duckieworkshop.tar```
```scp duckieworkshop.tar duckie@ROBOTNAME.local:~/```
at new robot:
```docker import duckieworkshop.tar```
```
docker run -d --restart unless-stopped --network=host \
  -w  /workspace \
  -e ROBOT_NAME=<ROBOTNAME> -e FRONTEND_PORT=8765 -e FRONTEND_DIR=/workspace/Workshop/WebFrontend -e ROS_DISTRO=noetic -e JUPYTER_PORT=8888 -e NOTEBOOK_DIR=/workspace/Workshop/Tasks \
  --name duckieworkshop duckieworkshop:latest \
  /usr/local/bin/start_services.sh
```