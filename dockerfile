FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive \
	PYTHONDONTWRITEBYTECODE=1 \
	PYTHONUNBUFFERED=1

WORKDIR /workspace

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-dev \
    pkg-config \
	nano

RUN apt-get update && apt-get install -y --no-install-recommends \
	python3-pip \
	python3-opencv \
	python3-numpy \
	python3-rospy \
	iproute2 \
	ca-certificates \
	curl \
	&& rm -rf /var/lib/apt/lists/* \
    ros-noetic-cv-bridge

RUN python3 -m pip install --upgrade pip setuptools wheel

RUN pip3 install \
    pyzmq==26.0.3

RUN pip3 install --no-cache-dir \
	jupyterlab==4.2.5 \
	notebook==7.2.2 \
	cv_bridge \
	ipyturtle3


# Build the ROS message package once at image build time. It is copied on its
# own first so that code changes elsewhere in Workshop don't trigger a rebuild.
COPY Workshop/Duckietown_msg ./Workshop/Duckietown_msg
RUN bash -c "source /opt/ros/noetic/setup.bash \
	&& cd /workspace/Workshop/Duckietown_msg \
	&& catkin_make"

# Shells opened with docker exec get ROS and the message package as well
RUN echo "source /workspace/Workshop/Duckietown_msg/devel/setup.bash" >> /root/.bashrc

COPY Workshop ./Workshop
COPY scripts/start_services.sh /usr/local/bin/start_services.sh


RUN chmod +x /usr/local/bin/start_services.sh

ENV NOTEBOOK_DIR=/workspace/Workshop/Tasks \
	FRONTEND_DIR=/workspace/Workshop/WebFrontend \
	JUPYTER_PORT=8888 \
	FRONTEND_PORT=8765 \
	BRIDGE_PATH=/ros

EXPOSE 8888 8765

HEALTHCHECK --interval=20s --timeout=5s --start-period=25s --retries=3 \
	CMD curl -fsS "http://127.0.0.1:${JUPYTER_PORT}/lab" >/dev/null || exit 1


# Services run in the foreground, so the container also works detached
# (docker run -d --restart unless-stopped) and starts on robot boot.
# Open a shell with: docker exec -it duckieworkshop bash
ENTRYPOINT ["/usr/local/bin/start_services.sh"]
