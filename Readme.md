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

Build Docker (requires Internet Connection and will download for some time)
```docker build -t duckieworkshop:latest . ```
```docker run -it --network=host --name duckieworkshop duckieworkshop:latest```

Start Docker container
```docker start -i duckieworkshop```

If testing (including ros master in the docker don't do it on the actual duckiebot)
```docker run -it --network host -e ROS_MASTER_URI=http://tick.local:11311 --name duckieworkshop duckieworkshop:latest```

Copying docker image from robot to robot (Only a quick fix) (all these commands take a long time)
```docker export duckieworkshop duckieworkshop.tar```
```scp duckieworkshop.tar duckie@ROBOTNAME.local:~/```
at new robot:
```docker import duckieworkshop.tar```
```
docker run -it --network=host \
  -w  /workspace \
  -e ROBOT_NAME=<ROBOTNAME> -e FRONTEND_PORT=8765 -e FRONTEND_DIR=/workspace/Workshop/WebFrontend -e ROS_DISTRO=noetic -e JUPYTER_PORT=8888 \
  --name duckieworkshop duckieworkshop:latest \
  bash -c "/usr/local/bin/start_services.sh & exec bash" 
```