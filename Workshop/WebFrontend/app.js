const sliderDefinitions = {
  yellow: [
    { key: "h_low", label: "H Low", min: 0, max: 255, step: 1, value: 20 },
    { key: "h_high", label: "H High", min: 0, max: 255, step: 1, value: 40 },
    { key: "s_low", label: "S Low", min: 0, max: 255, step: 1, value: 80 },
    { key: "s_high", label: "S High", min: 0, max: 255, step: 1, value: 255 },
    { key: "v_low", label: "V Low", min: 0, max: 255, step: 1, value: 120 },
    { key: "v_high", label: "V High", min: 0, max: 255, step: 1, value: 255 }
  ],
  white: [
    { key: "h_low", label: "H Low", min: 0, max: 255, step: 1, value: 0 },
    { key: "h_high", label: "H High", min: 0, max: 255, step: 1, value: 255 },
    { key: "s_low", label: "S Low", min: 0, max: 255, step: 1, value: 0 },
    { key: "s_high", label: "S High", min: 0, max: 255, step: 1, value: 60 },
    { key: "v_low", label: "V Low", min: 0, max: 255, step: 1, value: 180 },
    { key: "v_high", label: "V High", min: 0, max: 255, step: 1, value: 255 }
  ],
  pid: [
    { key: "p", label: "P", min: 0, max: 20, step: 0.05, value: 6.0 },
    { key: "i", label: "I", min: 0, max: 5, step: 0.01, value: 0.0 },
    { key: "d", label: "D", min: 0, max: 5, step: 0.01, value: 0.1 }
  ]
};

const elements = {
  connectionStatus: document.getElementById("connectionStatus"),
  endpointLabel: document.getElementById("endpointLabel"),
  cameraTopicLabel: document.getElementById("cameraTopicLabel"),
  cameraImage: document.getElementById("cameraImage"),
  cameraPlaceholder: document.getElementById("cameraPlaceholder"),
  cameraActive: document.getElementById("cameraActive"),
  startButton: document.getElementById("startButton"),
  stopButton: document.getElementById("stopButton"),
  publishHsvButton: document.getElementById("publishHsvButton"),
  publishPidButton: document.getElementById("publishPidButton"),
  yellowControls: document.getElementById("yellowControls"),
  whiteControls: document.getElementById("whiteControls"),
  pidControls: document.getElementById("pidControls"),
  clearLogButton: document.getElementById("clearLogButton"),
  logOutput: document.getElementById("logOutput")
};

const state = {
  robotName: "",
  wsPort: 9001,
  socket: null,
  status: "offline",
  reconnectDelayMs: 1000,
  maxReconnectDelayMs: 8000,
  reconnectTimer: null,
  subscribedCamera: false,
  debounceTimers: new Map(),
  topics: null
};

function getRobotName() {
  return state.robotName;
}

function getEndpoint() {
  const robotName = getRobotName() || "-";
  return `ws://${robotName}.local:${state.wsPort}`;
}

function getTopics() {
  const robotName = getRobotName();
  return {
    camera: `/${robotName}/camera_node/image/compressed`,
    start: `/${robotName}/start`,
    yellow: `/${robotName}/lane_detection_node/yellow`,
    white: `/${robotName}/lane_detection_node/white`,
    pid: `/${robotName}/lane_control_node/pid`
  };
}

function setStatus(status) {
  state.status = status;
  elements.connectionStatus.textContent = status;
  elements.connectionStatus.className = `status status-${status}`;

  const enabled = status === "online";
  elements.startButton.disabled = !enabled;
  elements.stopButton.disabled = !enabled;
  elements.publishHsvButton.disabled = !enabled;
  elements.publishPidButton.disabled = !enabled;

  document.querySelectorAll("input[type='range']").forEach((input) => {
    input.disabled = !enabled;
  });
}

function updateLabels() {
  const robotName = getRobotName() || "robot_name";
  elements.endpointLabel.textContent = getEndpoint();
  const topics = getTopics();
  elements.cameraTopicLabel.textContent = topics.camera;
  state.topics = topics;
}

function log(message) {
  const line = `[${new Date().toLocaleTimeString()}] ${message}`;
  const current = elements.logOutput.textContent.trim();
  elements.logOutput.textContent = current ? `${line}\n${current}` : line;
}

function makeHeader() {
  return {
    seq: 0,
    stamp: { secs: 0, nsecs: 0 },
    frame_id: ""
  };
}

function send(payload) {
  if (!state.socket || state.socket.readyState !== WebSocket.OPEN) {
    log("WebSocket is not connected.");
    return false;
  }

  state.socket.send(JSON.stringify(payload));
  return true;
}

function publish(topic, msg, latch = false) {
  const success = send({ op: "publish", topic, msg, latch });
  if (success) {
    log(`Published to ${topic}`);
  }
}

function subscribeCamera() {
  if (!state.topics || state.subscribedCamera || !elements.cameraActive.checked) {
    return;
  }

  const success = send({
    op: "subscribe",
    topic: state.topics.camera,
    type: "sensor_msgs/CompressedImage",
    throttle_rate: 150,
    queue_length: 1
  });

  if (success) {
    state.subscribedCamera = true;
    log(`Subscribed to ${state.topics.camera}`);
  }
}

function unsubscribeCamera() {
  if (!state.topics || !state.subscribedCamera) {
    return;
  }

  const success = send({ op: "unsubscribe", topic: state.topics.camera });
  if (success) {
    state.subscribedCamera = false;
    log(`Unsubscribed from ${state.topics.camera}`);
  }
}

function scheduleReconnect() {
  if (state.reconnectTimer) {
    return;
  }

  setStatus("connecting");
  state.reconnectTimer = window.setTimeout(() => {
    state.reconnectTimer = null;
    connect();
  }, state.reconnectDelayMs);

  state.reconnectDelayMs = Math.min(state.reconnectDelayMs * 2, state.maxReconnectDelayMs);
}

function teardownSocket() {
  if (!state.socket) {
    return;
  }

  state.socket.onopen = null;
  state.socket.onclose = null;
  state.socket.onerror = null;
  state.socket.onmessage = null;
  try {
    state.socket.close();
  } catch (error) {
    log(`Socket close error: ${error.message}`);
  }
  state.socket = null;
  state.subscribedCamera = false;
}

function connect() {
  const robotName = getRobotName();
  if (!robotName) {
    log("Enter a robot name first.");
    return;
  }

  updateLabels();
  teardownSocket();
  setStatus("connecting");
  log(`Connecting to ${getEndpoint()}`);

  const socket = new WebSocket(getEndpoint());
  state.socket = socket;

  socket.onopen = () => {
    state.reconnectDelayMs = 1000;
    setStatus("online");
    log("WebSocket connected.");
    subscribeCamera();
  };

  socket.onclose = () => {
    log("WebSocket disconnected.");
    state.subscribedCamera = false;
    setStatus("offline");
    if (state.robotName) {
      scheduleReconnect();
    }
  };

  socket.onerror = () => {
    log("WebSocket error.");
  };

  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    handleMessage(message);
  };
}

function disconnect() {
  if (state.reconnectTimer) {
    clearTimeout(state.reconnectTimer);
    state.reconnectTimer = null;
  }
  teardownSocket();
  setStatus("offline");
  log("Disconnected by user.");
}

function handleMessage(message) {
  if (message.op === "publish" && state.topics && message.topic === state.topics.camera) {
    const imageData = message.msg?.data;
    if (imageData) {
      elements.cameraImage.src = `data:image/jpeg;base64,${imageData}`;
      elements.cameraImage.style.display = "block";
      elements.cameraPlaceholder.style.display = "none";
    }
    return;
  }

  if (message.op === "service_response") {
    log(`Service response: ${message.service || message.id || "unknown"}`);
    return;
  }

  if (message.op === "status") {
    log(`Rosbridge status: ${message.msg}`);
  }
}

function getSectionValues(section) {
  const values = { header: makeHeader() };
  sliderDefinitions[section].forEach((definition) => {
    const input = document.getElementById(`${section}-${definition.key}`);
    const rawValue = Number(input.value);
    values[definition.key] = definition.step < 1 ? rawValue : Math.round(rawValue);
  });
  return values;
}

function debounce(key, fn, delayMs) {
  if (state.debounceTimers.has(key)) {
    clearTimeout(state.debounceTimers.get(key));
  }
  const timer = window.setTimeout(() => {
    state.debounceTimers.delete(key);
    fn();
  }, delayMs);
  state.debounceTimers.set(key, timer);
}

function publishHsvTopics() {
  if (!state.topics) {
    return;
  }
  publish(state.topics.yellow, getSectionValues("yellow"));
  publish(state.topics.white, getSectionValues("white"));
}

function publishPidTopic() {
  if (!state.topics) {
    return;
  }
  publish(state.topics.pid, getSectionValues("pid"));
}

function publishStart(value) {
  if (!state.topics) {
    return;
  }
  publish(state.topics.start, { data: value });
}

function makeSlider(section, definition) {
  const row = document.createElement("label");
  row.className = "slider-row";

  const title = document.createElement("span");
  title.textContent = definition.label;

  const input = document.createElement("input");
  input.type = "range";
  input.id = `${section}-${definition.key}`;
  input.min = definition.min;
  input.max = definition.max;
  input.step = definition.step;
  input.value = definition.value;
  input.disabled = true;

  const output = document.createElement("output");
  output.htmlFor = input.id;
  output.textContent = String(definition.value);

  input.addEventListener("input", () => {
    output.textContent = input.value;
    if (section === "pid") {
      debounce(`pid-${definition.key}`, publishPidTopic, 200);
    } else {
      debounce(`${section}-${definition.key}`, publishHsvTopics, 200);
    }
  });

  row.append(title, input, output);
  return row;
}

function mountSliders() {
  sliderDefinitions.yellow.forEach((definition) => {
    elements.yellowControls.appendChild(makeSlider("yellow", definition));
  });
  sliderDefinitions.white.forEach((definition) => {
    elements.whiteControls.appendChild(makeSlider("white", definition));
  });
  sliderDefinitions.pid.forEach((definition) => {
    elements.pidControls.appendChild(makeSlider("pid", definition));
  });
}

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

function bootstrap() {
  const params = new URLSearchParams(window.location.search);
  const robot = params.get("robot");
  const port = Number(params.get("port"));

  state.robotName = (robot || inferRobotNameFromHost()).toLowerCase();
  if (Number.isFinite(port) && port > 0) {
    state.wsPort = port;
  }

  updateLabels();
}

elements.startButton.addEventListener("click", () => publishStart(1));
elements.stopButton.addEventListener("click", () => publishStart(0));
elements.publishHsvButton.addEventListener("click", publishHsvTopics);
elements.publishPidButton.addEventListener("click", publishPidTopic);
elements.clearLogButton.addEventListener("click", () => {
  elements.logOutput.textContent = "";
});
elements.cameraActive.addEventListener("change", () => {
  if (elements.cameraActive.checked) {
    subscribeCamera();
  } else {
    unsubscribeCamera();
  }
});

mountSliders();
bootstrap();
setStatus("offline");
log(`Ready on ${state.robotName}. Attempting automatic connection.`);
connect();