const state = {
  robotName: "",
  wsPort: 9001,
  socket: null,
  image: null,
  scale: 1,
  offsetX: 0,
  offsetY: 0,
  dragging: false,
  lastPointerX: 0,
  lastPointerY: 0,
  hasFittedImage: false
};

const elements = {
  status: document.getElementById("connectionStatus"),
  endpoint: document.getElementById("endpointLabel"),
  topic: document.getElementById("mapTopic"),
  connectButton: document.getElementById("connectButton"),
  fitButton: document.getElementById("fitButton"),
  resetButton: document.getElementById("resetButton"),
  info: document.getElementById("mapInfo"),
  frame: document.getElementById("mapFrame"),
  canvas: document.getElementById("mapCanvas")
};

const context = elements.canvas.getContext("2d");

function inferRobotNameFromHost() {
  const host = window.location.hostname || "";

  if (host.endsWith(".local")) {
    return host.slice(0, -6);
  }

  if (host === "localhost" || host === "127.0.0.1") {
    return "tick";
  }

  return host || "tick";
}

function getEndpoint() {
  return `ws://${state.robotName}.local:${state.wsPort}`;
}

function getMapTopic() {
  return `/${state.robotName}/map_image`;
}

function setStatus(status) {
  elements.status.textContent = status;
  elements.status.className = `status status-${status}`;
}

function resizeCanvas() {
  const rectangle = elements.frame.getBoundingClientRect();
  const devicePixelRatio = window.devicePixelRatio || 1;

  elements.canvas.width = Math.round(rectangle.width * devicePixelRatio);
  elements.canvas.height = Math.round(rectangle.height * devicePixelRatio);

  context.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  drawMap();
}

function drawMap() {
  const width = elements.frame.clientWidth;
  const height = elements.frame.clientHeight;

  context.clearRect(0, 0, width, height);
  context.fillStyle = "#111827";
  context.fillRect(0, 0, width, height);

  if (!state.image) {
    context.fillStyle = "#cbd5e1";
    context.font = "16px Segoe UI, sans-serif";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText("No map image received yet.", width / 2, height / 2);
    return;
  }

  context.save();
  context.translate(state.offsetX, state.offsetY);
  context.scale(state.scale, state.scale);
  context.imageSmoothingEnabled = false;
  context.drawImage(state.image, 0, 0);
  context.restore();
}

function fitMap() {
  if (!state.image) {
    return;
  }

  const width = elements.frame.clientWidth;
  const height = elements.frame.clientHeight;
  const scaleX = width / state.image.width;
  const scaleY = height / state.image.height;

  state.scale = Math.min(scaleX, scaleY) * 0.95;
  state.offsetX = (width - state.image.width * state.scale) / 2;
  state.offsetY = (height - state.image.height * state.scale) / 2;

  drawMap();
}

function resetMap() {
  state.scale = 1;
  state.offsetX = 0;
  state.offsetY = 0;
  drawMap();
}

function getImageMimeType(message) {
  const format = String(message.msg?.format || "").toLowerCase();

  if (format.includes("png")) {
    return "image/png";
  }

  if (format.includes("webp")) {
    return "image/webp";
  }

  return "image/jpeg";
}

function handleMapMessage(message) {
  if (message.op !== "publish") {
    return;
  }

  if (message.topic !== elements.topic.value.trim()) {
    return;
  }

  const imageData = message.msg?.data;
  if (!imageData) {
    return;
  }

  const image = new Image();

  image.onload = () => {
    state.image = image;
    elements.info.textContent = `Image received: ${image.width} x ${image.height}px`;

    if (!state.hasFittedImage) {
      state.hasFittedImage = true;
      fitMap();
    } else {
      drawMap();
    }
  };

  image.onerror = () => {
    elements.info.textContent = "The map image could not be loaded.";
  };

  image.src = `data:${getImageMimeType(message)};base64,${imageData}`;
}

function subscribeMap() {
  const topic = elements.topic.value.trim();

  if (!topic || !state.socket) {
    return;
  }

  state.socket.send(JSON.stringify({
    op: "subscribe",
    topic,
    type: "sensor_msgs/CompressedImage",
    throttle_rate: 250,
    queue_length: 1
  }));

  elements.info.textContent = `Subscribed: ${topic}`;
}

function connect() {
  if (state.socket) {
    state.socket.close();
    state.socket = null;
  }

  state.hasFittedImage = false;
  setStatus("connecting");
  elements.info.textContent = `Connecting to ${getEndpoint()} ...`;

  const socket = new WebSocket(getEndpoint());
  state.socket = socket;

  socket.onopen = () => {
    setStatus("online");
    subscribeMap();
  };

  socket.onmessage = (event) => {
    try {
      handleMapMessage(JSON.parse(event.data));
    } catch {
      elements.info.textContent = "Received an invalid WebSocket message.";
    }
  };

  socket.onerror = () => {
    setStatus("offline");
    elements.info.textContent = "WebSocket error.";
  };

  socket.onclose = () => {
    if (state.socket === socket) {
      state.socket = null;
      setStatus("offline");
    }
  };
}

function initialize() {
  const params = new URLSearchParams(window.location.search);

  state.robotName = (
    params.get("robot") || inferRobotNameFromHost()
  ).toLowerCase();

  const port = Number(params.get("port"));
  if (Number.isFinite(port) && port > 0) {
    state.wsPort = port;
  }

  elements.topic.value = params.get("topic") || getMapTopic();
  elements.endpoint.textContent = getEndpoint();
  resizeCanvas();
}

elements.connectButton.addEventListener("click", connect);
elements.fitButton.addEventListener("click", fitMap);
elements.resetButton.addEventListener("click", resetMap);

elements.frame.addEventListener("wheel", (event) => {
  event.preventDefault();

  if (!state.image) {
    return;
  }

  const rectangle = elements.frame.getBoundingClientRect();
  const mouseX = event.clientX - rectangle.left;
  const mouseY = event.clientY - rectangle.top;
  const oldScale = state.scale;
  const zoomFactor = event.deltaY < 0 ? 1.15 : 0.85;
  const newScale = Math.min(Math.max(oldScale * zoomFactor, 0.05), 20);

  state.offsetX = mouseX - (mouseX - state.offsetX) * (newScale / oldScale);
  state.offsetY = mouseY - (mouseY - state.offsetY) * (newScale / oldScale);
  state.scale = newScale;

  drawMap();
}, { passive: false });

elements.frame.addEventListener("pointerdown", (event) => {
  state.dragging = true;
  state.lastPointerX = event.clientX;
  state.lastPointerY = event.clientY;
  elements.frame.setPointerCapture(event.pointerId);
});

elements.frame.addEventListener("pointermove", (event) => {
  if (!state.dragging) {
    return;
  }

  state.offsetX += event.clientX - state.lastPointerX;
  state.offsetY += event.clientY - state.lastPointerY;
  state.lastPointerX = event.clientX;
  state.lastPointerY = event.clientY;

  drawMap();
});

function stopDragging() {
  state.dragging = false;
}

elements.frame.addEventListener("pointerup", stopDragging);
elements.frame.addEventListener("pointercancel", stopDragging);
window.addEventListener("resize", resizeCanvas);

initialize();
connect();
