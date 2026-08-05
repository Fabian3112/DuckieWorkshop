FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive \
	PYTHONDONTWRITEBYTECODE=1 \
	PYTHONUNBUFFERED=1

WORKDIR /workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
	python3-pip \
	python3-opencv \
	python3-numpy \
	python3-rospy \
	iproute2 \
	ca-certificates \
	curl \
	&& rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
	jupyterlab==4.2.5 \
	notebook==7.2.2 \
	cv-bridge

COPY Workshop ./Workshop
COPY scripts/start_services.sh /usr/local/bin/start_services.sh
COPY scripts/test_setup.sh /usr/local/bin/test_setup.sh


RUN chmod +x /usr/local/bin/start_services.sh
RUN chmod +x /usr/local/bin/test_setup.sh

ENV NOTEBOOK_DIR=/workspace/Workshop/Tasks \
	FRONTEND_DIR=/workspace/Workshop/WebFrontend \
	JUPYTER_PORT=8888 \
	FRONTEND_PORT=8000 \
	BRIDGE_PATH=/ros

HEALTHCHECK --interval=20s --timeout=5s --start-period=25s --retries=3 \
	CMD curl -fsS "http://127.0.0.1:${JUPYTER_PORT}/lab" >/dev/null || exit 1


ENTRYPOINT ["bash", "-c", "/usr/local/bin/start_services.sh & exec bash"]
