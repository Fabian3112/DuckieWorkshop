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
```docker run -p 8081:80 -p 8888:8888 --name duckieworkshop duckieworkshop:latest```

If testing (including ros master in the docker don't do it on the actual duckiebot)
```docker run -it --network host -e ROS_MASTER_URI=http://tick.local:11311 --name duckieworkshop duckieworkshop:latest```