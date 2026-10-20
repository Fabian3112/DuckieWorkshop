// Must match the defaults in Robot/duckie_detector.py. OpenCV hue goes from 0 to 180.
// Hue may wrap: if low is bigger than high, everything above low or below high counts.
// Red is at both ends of the hue circle, so the default wraps.
const hsvDefinitions = [
  { key: "h", label: "Hue", min: 0, max: 180, low: 170, high: 10, wrap: true },
  { key: "s", label: "Sat", min: 0, max: 255, low: 120, high: 255 },
  { key: "v", label: "Val", min: 0, max: 255, low: 80, high: 255 }
];

const shapeDefinitions = [
  { key: "min_area", label: "Min size", min: 0, max: 5000, step: 10, value: 300 },
  { key: "trigger_line", label: "Trigger line", min: 0, max: 200, step: 1, value: 150 }
];

const elements = {
  status: document.getElementById("connectionStatus"),
  endpoint: document.getElementById("endpointLabel"),
  topicLabel: document.getElementById("duckTopicLabel"),
  image: document.getElementById("duckImage"),
  placeholder: document.getElementById("duckPlaceholder"),
  publishButton: document.getElementById("publishButton"),
  hsvControls: document.getElementById("hsvControls"),
  shapeControls: document.getElementById("shapeControls"),
  evadeActive: document.getElementById("evadeActive")
};

const state = {
  robotName: "",
  wsPort: 9001,
  socket: null,
  advertised: false,
  publishTimer: null
};

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

function getImageTopic() {
  return `/${state.robotName}/robot/duck_image/compressed`;
}

function getSettingsTopic() {
  return `/${state.robotName}/duck_detection_node/hsv`;
}

function setStatus(status) {
  elements.status.textContent = status;
  elements.status.className = `status status-${status}`;

  const enabled = status === "online";
  elements.publishButton.disabled = !enabled;
  elements.evadeActive.disabled = !enabled;
  document.querySelectorAll("input[type='range']").forEach((input) => {
    input.disabled = !enabled;
  });
}

function getSettings() {
  const values = {};
  hsvDefinitions.forEach((definition) => {
    values[`${definition.key}_low`] = Number(document.getElementById(`duck-${definition.key}_low`).value);
    values[`${definition.key}_high`] = Number(document.getElementById(`duck-${definition.key}_high`).value);
  });
  shapeDefinitions.forEach((definition) => {
    values[definition.key] = Number(document.getElementById(`duck-${definition.key}`).value);
  });
  values.evade = elements.evadeActive.checked;
  return values;
}

function publishSettings() {
  if (!state.socket || state.socket.readyState !== WebSocket.OPEN) {
    return;
  }

  const topic = getSettingsTopic();
  if (!state.advertised) {
    state.socket.send(JSON.stringify({ op: "advertise", topic, type: "std_msgs/String" }));
    state.advertised = true;
  }
  state.socket.send(JSON.stringify({ op: "publish", topic, msg: { data: JSON.stringify(getSettings()) } }));
}

// Publish while the slider is moving, but at most every 100 ms
function schedulePublish() {
  if (state.publishTimer) {
    return;
  }
  state.publishTimer = window.setTimeout(() => {
    state.publishTimer = null;
    publishSettings();
  }, 100);
}

function makeDualRangeSlider(definition) {
  const row = document.createElement("div");
  row.className = "slider-row slider-row-dual";

  const title = document.createElement("span");
  title.textContent = definition.label;

  const rangeWrap = document.createElement("div");
  rangeWrap.className = "dual-range";

  const track = document.createElement("div");
  track.className = "dual-track";

  const lowInput = document.createElement("input");
  lowInput.type = "range";
  lowInput.id = `duck-${definition.key}_low`;
  lowInput.min = definition.min;
  lowInput.max = definition.max;
  lowInput.step = 1;
  lowInput.value = definition.low;
  lowInput.disabled = true;

  const highInput = document.createElement("input");
  highInput.type = "range";
  highInput.id = `duck-${definition.key}_high`;
  highInput.min = definition.min;
  highInput.max = definition.max;
  highInput.step = 1;
  highInput.value = definition.high;
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

    if (low <= high) {
      track.style.background = `linear-gradient(to right, #dbe7f8 0% ${lowPercent}%, #1d4ed8 ${lowPercent}% ${highPercent}%, #dbe7f8 ${highPercent}% 100%)`;
    } else {
      // wrapped range: selected from 0 to high and from low to the end
      track.style.background = `linear-gradient(to right, #1d4ed8 0% ${highPercent}%, #dbe7f8 ${highPercent}% ${lowPercent}%, #1d4ed8 ${lowPercent}% 100%)`;
    }
    lowOutput.textContent = `L:${low}`;
    highOutput.textContent = `H:${high}`;
  }

  lowInput.addEventListener("input", () => {
    if (!definition.wrap && Number(lowInput.value) > Number(highInput.value)) {
      lowInput.value = highInput.value;
    }
    updateTrackAndValues();
    schedulePublish();
  });

  highInput.addEventListener("input", () => {
    if (!definition.wrap && Number(highInput.value) < Number(lowInput.value)) {
      highInput.value = lowInput.value;
    }
    updateTrackAndValues();
    schedulePublish();
  });

  updateTrackAndValues();

  rangeWrap.append(track, lowInput, highInput);
  valuesWrap.append(lowOutput, highOutput);
  row.append(title, rangeWrap, valuesWrap);
  return row;
}

function makeSingleSlider(definition) {
  const row = document.createElement("label");
  row.className = "slider-row";

  const title = document.createElement("span");
  title.textContent = definition.label;

  const input = document.createElement("input");
  input.type = "range";
  input.id = `duck-${definition.key}`;
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
    schedulePublish();
  });

  row.append(title, input, output);
  return row;
}

function mountSliders() {
  hsvDefinitions.forEach((definition) => {
    elements.hsvControls.appendChild(makeDualRangeSlider(definition));
  });
  shapeDefinitions.forEach((definition) => {
    elements.shapeControls.appendChild(makeSingleSlider(definition));
  });
}

function handleMessage(message) {
  if (message.op !== "publish" || message.topic !== getImageTopic()) {
    return;
  }

  const imageData = message.msg?.data;
  if (imageData) {
    elements.image.src = `data:image/jpeg;base64,${imageData}`;
    elements.image.style.display = "block";
    elements.placeholder.style.display = "none";
  }
}

function connect() {
  if (state.socket) {
    state.socket.onclose = null;
    state.socket.close();
  }

  setStatus("connecting");
  state.advertised = false;

  const socket = new WebSocket(getEndpoint());
  state.socket = socket;

  socket.onopen = () => {
    setStatus("online");
    socket.send(JSON.stringify({
      op: "subscribe",
      topic: getImageTopic(),
      type: "sensor_msgs/CompressedImage",
      throttle_rate: 100,
      queue_length: 1
    }));
  };

  socket.onclose = () => {
    setStatus("offline");
    window.setTimeout(connect, 2000);
  };

  socket.onmessage = (event) => {
    try {
      handleMessage(JSON.parse(event.data));
    } catch (error) {
      // ignore broken messages
    }
  };
}

function bootstrap() {
  const params = new URLSearchParams(window.location.search);
  const robot = params.get("robot");
  const port = Number(params.get("port"));

  state.robotName = (robot || inferRobotNameFromHost()).toLowerCase();
  if (Number.isFinite(port) && port > 0) {
    state.wsPort = port;
  }

  elements.endpoint.textContent = getEndpoint();
  elements.topicLabel.textContent = getImageTopic();
}

elements.publishButton.addEventListener("click", publishSettings);
elements.evadeActive.addEventListener("change", publishSettings);

mountSliders();
bootstrap();
setStatus("offline");
connect();
