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
    { key: "d", label: "D", min: 0, max: 5, step: 0.01, value: 0.1 },
    { key: "speed", label: "Speed", min: 0, max: 1, step: 0.01, value: 0.2 }
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
  logOutput: document.getElementById("logOutput"),
  programStatus: document.getElementById("programStatus"),
  programButton: document.getElementById("programButton"),
  programOutput: document.getElementById("programOutput"),
  optionIntersection: document.getElementById("optionIntersection"),
  optionDuckEvasion: document.getElementById("optionDuckEvasion")
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
  advertisedTopics: new Set(),
  debounceTimers: new Map(),
  topics: null,
  savedSettings: {},
  program: { running: false, busy: false, reachable: false }
};

// The robot stores the last parameters (robot_server.py), so they survive page changes and restarts
async function loadSavedSettings() {
  try {
    const response = await fetch("/api/settings", { cache: "no-store" });
    if (response.ok) {
      state.savedSettings = await response.json();
    }
  } catch (error) {
    log("Could not load the saved parameters.");
  }

  ["yellow", "white", "pid"].forEach((section) => {
    const saved = state.savedSettings[section];
    if (!saved) {
      return;
    }
    sliderDefinitions[section].forEach((definition) => {
      const value = Number(saved[definition.key]);
      if (Number.isFinite(value)) {
        definition.value = Math.min(Math.max(value, definition.min), definition.max);
      }
    });
  });

  const start = state.savedSettings.start || {};
  elements.optionIntersection.checked = Boolean(start.intersection);
  elements.optionDuckEvasion.checked = state.savedSettings.duck?.evade ?? start.duck_evasion ?? true;
}

function saveSettings(sections) {
  Object.assign(state.savedSettings, sections);
  fetch("/api/settings", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(sections)
  }).catch(() => log("Could not save the parameters on the robot."));
}

function withoutHeader(values) {
  const { header, ...rest } = values;
  return rest;
}

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
    camera: `/${robotName}/robot/image/compressed`,
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
  ensureAdvertised(topic);
  const success = send({ op: "publish", topic, msg, latch });
  if (success) {
    log(`Published to ${topic}`);
  }
}

function getTopicType(topic) {
  if (!state.topics) {
    return null;
  }

  if (topic === state.topics.start) {
    return "std_msgs/Int32";
  }
  if (topic === state.topics.yellow || topic === state.topics.white) {
    return "std_msgs/String";
  }
  if (topic === state.topics.pid) {
    return "std_msgs/String";
  }
  return null;
}

function ensureAdvertised(topic) {
  if (state.advertisedTopics.has(topic)) {
    return;
  }

  const type = getTopicType(topic);
  if (!type) {
    return;
  }

  const success = send({ op: "advertise", topic, type });
  if (success) {
    state.advertisedTopics.add(topic);
    log(`Advertised ${topic} as ${type}`);
  }
}

function advertiseControlTopics() {
  if (!state.topics) {
    return;
  }

  ensureAdvertised(state.topics.start);
  ensureAdvertised(state.topics.yellow);
  ensureAdvertised(state.topics.white);
  ensureAdvertised(state.topics.pid);
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
  state.advertisedTopics.clear();
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
    advertiseControlTopics();
    subscribeCamera();
    publishSavedSettings();
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
  const yellow = getSectionValues("yellow");
  const white = getSectionValues("white");
  publish(state.topics.yellow, { data: JSON.stringify(yellow) });
  publish(state.topics.white, { data: JSON.stringify(white) });
  saveSettings({ yellow: withoutHeader(yellow), white: withoutHeader(white) });
}

function publishPidTopic() {
  if (!state.topics) {
    return;
  }
  const pid = getSectionValues("pid");
  publish(state.topics.pid, { data: JSON.stringify(pid) });
  saveSettings({ pid: withoutHeader(pid) });
}

// a robot program started in a notebook gets the saved parameters as well
function publishSavedSettings() {
  if (state.savedSettings.yellow || state.savedSettings.white) {
    publishHsvTopics();
  }
  if (state.savedSettings.pid) {
    publishPidTopic();
  }
}

function publishStart(value) {
  if (!state.topics) {
    return;
  }
  publish(state.topics.start, { data: value });
}

function makeSingleSlider(section, definition) {
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

function makeDualRangeSlider(section, channelKey, label, lowDefinition, highDefinition) {
  const row = document.createElement("div");
  row.className = "slider-row slider-row-dual";

  const title = document.createElement("span");
  title.textContent = label;

  const rangeWrap = document.createElement("div");
  rangeWrap.className = "dual-range";

  const track = document.createElement("div");
  track.className = "dual-track";

  const lowInput = document.createElement("input");
  lowInput.type = "range";
  lowInput.id = `${section}-${channelKey}_low`;
  lowInput.min = lowDefinition.min;
  lowInput.max = lowDefinition.max;
  lowInput.step = lowDefinition.step;
  lowInput.value = lowDefinition.value;
  lowInput.disabled = true;

  const highInput = document.createElement("input");
  highInput.type = "range";
  highInput.id = `${section}-${channelKey}_high`;
  highInput.min = highDefinition.min;
  highInput.max = highDefinition.max;
  highInput.step = highDefinition.step;
  highInput.value = highDefinition.value;
  highInput.disabled = true;

  const valuesWrap = document.createElement("div");
  valuesWrap.className = "dual-values";

  const lowOutput = document.createElement("output");
  lowOutput.htmlFor = lowInput.id;

  const highOutput = document.createElement("output");
  highOutput.htmlFor = highInput.id;

  function updateTrackAndValues() {
    const min = Number(lowInput.min);
    const max = Number(lowInput.max);
    const low = Number(lowInput.value);
    const high = Number(highInput.value);
    const lowPercent = ((low - min) / (max - min)) * 100;
    const highPercent = ((high - min) / (max - min)) * 100;

    track.style.background = `linear-gradient(to right, #dbe7f8 0% ${lowPercent}%, #1d4ed8 ${lowPercent}% ${highPercent}%, #dbe7f8 ${highPercent}% 100%)`;
    lowOutput.textContent = `L:${low}`;
    highOutput.textContent = `H:${high}`;
  }

  lowInput.addEventListener("input", () => {
    if (Number(lowInput.value) > Number(highInput.value)) {
      lowInput.value = highInput.value;
    }
    updateTrackAndValues();
    debounce(`${section}-${channelKey}-pair`, publishHsvTopics, 200);
  });

  highInput.addEventListener("input", () => {
    if (Number(highInput.value) < Number(lowInput.value)) {
      highInput.value = lowInput.value;
    }
    updateTrackAndValues();
    debounce(`${section}-${channelKey}-pair`, publishHsvTopics, 200);
  });

  updateTrackAndValues();

  rangeWrap.append(track, lowInput, highInput);
  valuesWrap.append(lowOutput, highOutput);
  row.append(title, rangeWrap, valuesWrap);
  return row;
}

function appendHsvControls(section, targetElement) {
  const definitionMap = Object.fromEntries(
    sliderDefinitions[section].map((definition) => [definition.key, definition])
  );

  const channels = [
    { key: "h", label: "Hue" },
    { key: "s", label: "Sat" },
    { key: "v", label: "Val" }
  ];

  channels.forEach((channel) => {
    targetElement.appendChild(
      makeDualRangeSlider(
        section,
        channel.key,
        channel.label,
        definitionMap[`${channel.key}_low`],
        definitionMap[`${channel.key}_high`]
      )
    );
  });
}

function mountSliders() {
  appendHsvControls("yellow", elements.yellowControls);
  appendHsvControls("white", elements.whiteControls);
  sliderDefinitions.pid.forEach((definition) => {
    elements.pidControls.appendChild(makeSingleSlider("pid", definition));
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

function showProgram(status) {
  state.program.running = Boolean(status.running);
  state.program.reachable = true;

  let label = state.program.running ? "running" : "stopped";
  if (state.program.busy) {
    label = state.program.running ? "stopping" : "starting";
  }
  elements.programStatus.textContent = label;
  elements.programStatus.className = `status status-${state.program.running ? "online" : state.program.busy ? "connecting" : "offline"}`;

  elements.programButton.textContent = state.program.running ? "Stop Robot Program" : "Start Robot Program";
  elements.programButton.className = `button ${state.program.running ? "button-danger" : "button-primary"}`;
  elements.programButton.disabled = state.program.busy;

  // the options are chosen when the program starts
  elements.optionIntersection.disabled = state.program.running || state.program.busy;
  elements.optionDuckEvasion.disabled = state.program.running || state.program.busy;
  if (state.program.running && status.options) {
    elements.optionIntersection.checked = Boolean(status.options.intersection);
    elements.optionDuckEvasion.checked = Boolean(status.options.duck_evasion);
  }

  const output = (status.output || []).join("\n");
  if (elements.programOutput.textContent !== output) {
    elements.programOutput.textContent = output;
    elements.programOutput.scrollTop = elements.programOutput.scrollHeight;
  }
}

async function refreshProgram() {
  try {
    const response = await fetch("/api/robot", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(response.statusText);
    }
    showProgram(await response.json());
  } catch (error) {
    state.program.reachable = false;
    elements.programStatus.textContent = "unknown";
    elements.programStatus.className = "status status-connecting";
    elements.programButton.disabled = true;
  }
}

async function toggleProgram() {
  const action = state.program.running ? "stop" : "start";
  state.program.busy = true;
  showProgram({ running: state.program.running });
  log(action === "start" ? "Starting the robot program." : "Stopping the robot program.");

  try {
    const response = await fetch(`/api/robot/${action}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        intersection: elements.optionIntersection.checked,
        duck_evasion: elements.optionDuckEvasion.checked
      })
    });
    const status = await response.json();
    state.program.busy = false;
    showProgram(status);
  } catch (error) {
    state.program.busy = false;
    log(`Robot program could not be ${action === "start" ? "started" : "stopped"}: ${error.message}`);
    refreshProgram();
  }
}

elements.programButton.addEventListener("click", toggleProgram);
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

async function init() {
  await loadSavedSettings();
  mountSliders();
  bootstrap();
  setStatus("offline");
  log(`Ready on ${state.robotName}. Attempting automatic connection.`);
  connect();
  refreshProgram();
  window.setInterval(refreshProgram, 2000);
}

init();