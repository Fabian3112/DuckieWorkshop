FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive \
	PYTHONDONTWRITEBYTECODE=1 \
	PYTHONUNBUFFERED=1

WORKDIR /workspace

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-dev \
    pkg-config

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
	notebook==7.2.2


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


ENTRYPOINT ["bash", "-c", "/usr/local/bin/start_services.sh & exec bash"]
