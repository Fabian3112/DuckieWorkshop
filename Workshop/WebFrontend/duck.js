// Must match the defaults in Robot/duckie_detector.py. OpenCV hue goes from 0 to 180.
// Hue may wrap: if low is bigger than high, everything above low or below high counts.
// Red is at both ends of the hue circle, so the default wraps.
// Every colour is one HSV range, a pixel counts if it matches any of the colours.
const hsvDefinitions = [
  { key: "h", label: "Hue", min: 0, max: 180, wrap: true },
  { key: "s", label: "Sat", min: 0, max: 255 },
  { key: "v", label: "Val", min: 0, max: 255 }
];

const defaultColour = { h_low: 170, h_high: 10, s_low: 120, s_high: 255, v_low: 80, v_high: 255 };
const maxColours = 6;
const storageKey = "duckColours";

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
  colourList: document.getElementById("colourList"),
  addColourButton: document.getElementById("addColourButton"),
  shapeControls: document.getElementById("shapeControls"),
  evadeActive: document.getElementById("evadeActive")
};

const state = {
  robotName: "",
  wsPort: 9001,
  socket: null,
  advertised: false,
  publishTimer: null,
  online: false,
  colourBoxes: []
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

  state.online = status === "online";
  updateControls();
}

function updateControls() {
  const enabled = state.online;
  elements.publishButton.disabled = !enabled;
  elements.evadeActive.disabled = !enabled;
  elements.addColourButton.disabled = !enabled || state.colourBoxes.length >= maxColours;
  document.querySelectorAll("input[type='range']").forEach((input) => {
    input.disabled = !enabled;
  });
  state.colourBoxes.forEach((box, index) => {
    box.legend.textContent = `Colour ${index + 1}`;
    box.removeButton.disabled = !enabled || state.colourBoxes.length <= 1;
  });
}

function readColour(box) {
  const colour = {};
  hsvDefinitions.forEach((definition) => {
    colour[`${definition.key}_low`] = Number(box.inputs[`${definition.key}_low`].value);
    colour[`${definition.key}_high`] = Number(box.inputs[`${definition.key}_high`].value);
  });
  return colour;
}

function saveColours() {
  try {
    localStorage.setItem(storageKey, JSON.stringify(state.colourBoxes.map(readColour)));
  } catch (error) {
    // storage not available, the colours are just not remembered
  }
}

function loadColours() {
  try {
    const colours = JSON.parse(localStorage.getItem(storageKey));
    if (Array.isArray(colours) && colours.length > 0) {
      return colours.slice(0, maxColours);
    }
  } catch (error) {
    // nothing saved yet
  }
  return [defaultColour];
}

function getSettings() {
  const values = { colours: state.colourBoxes.map(readColour) };
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
  saveColours();
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

function makeDualRangeSlider(definition, idPrefix, low, high, onChange) {
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
  lowInput.id = `${idPrefix}-${definition.key}_low`;
  lowInput.min = definition.min;
  lowInput.max = definition.max;
  lowInput.step = 1;
  lowInput.value = low;
  lowInput.disabled = true;

  const highInput = document.createElement("input");
  highInput.type = "range";
  highInput.id = `${idPrefix}-${definition.key}_high`;
  highInput.min = definition.min;
  highInput.max = definition.max;
  highInput.step = 1;
  highInput.value = high;
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
    onChange();
  });

  highInput.addEventListener("input", () => {
    if (!definition.wrap && Number(highInput.value) < Number(lowInput.value)) {
      highInput.value = lowInput.value;
    }
    updateTrackAndValues();
    onChange();
  });

  updateTrackAndValues();

  rangeWrap.append(track, lowInput, highInput);
  valuesWrap.append(lowOutput, highOutput);
  row.append(title, rangeWrap, valuesWrap);
  return { row, lowInput, highInput };
}

// Shows roughly what the selected colour looks like
function updateSwatch(box) {
  const colour = readColour(box);
  let hue = (colour.h_low + colour.h_high) / 2;
  if (colour.h_low > colour.h_high) {
    hue = (hue + 90) % 180;
  }
  const saturation = (colour.s_low + colour.s_high) / 2 / 255;
  const value = (colour.v_low + colour.v_high) / 2 / 255;
  // HSV to HSL for CSS
  const lightness = value * (1 - saturation / 2);
  const hslSaturation = lightness === 0 || lightness === 1 ? 0 : (value - lightness) / Math.min(lightness, 1 - lightness);
  box.swatch.style.background = `hsl(${hue * 2}, ${Math.round(hslSaturation * 100)}%, ${Math.round(lightness * 100)}%)`;
}

function addColour(colour) {
  const id = `duck-colour-${Date.now()}-${Math.round(Math.random() * 1000)}`;

  const fieldset = document.createElement("fieldset");
  fieldset.className = "colour-box";

  const legend = document.createElement("legend");
  const swatch = document.createElement("span");
  swatch.className = "colour-swatch";

  const removeButton = document.createElement("button");
  removeButton.type = "button";
  removeButton.className = "button button-secondary colour-remove";
  removeButton.textContent = "Remove";

  const header = document.createElement("div");
  header.className = "colour-header";
  header.append(swatch, removeButton);

  const stack = document.createElement("div");
  stack.className = "slider-stack";

  const box = { fieldset, legend, swatch, removeButton, inputs: {} };

  hsvDefinitions.forEach((definition) => {
    const slider = makeDualRangeSlider(
      definition,
      id,
      colour[`${definition.key}_low`],
      colour[`${definition.key}_high`],
      () => {
        updateSwatch(box);
        schedulePublish();
      }
    );
    box.inputs[`${definition.key}_low`] = slider.lowInput;
    box.inputs[`${definition.key}_high`] = slider.highInput;
    stack.appendChild(slider.row);
  });

  removeButton.addEventListener("click", () => {
    if (state.colourBoxes.length <= 1) {
      return;
    }
    state.colourBoxes = state.colourBoxes.filter((other) => other !== box);
    fieldset.remove();
    updateControls();
    publishSettings();
  });

  fieldset.append(legend, header, stack);
  elements.colourList.appendChild(fieldset);
  state.colourBoxes.push(box);
  updateSwatch(box);
  updateControls();
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
  loadColours().forEach((colour) => addColour({ ...defaultColour, ...colour }));
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
    // the robot gets the colours shown on this page
    publishSettings();
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
elements.addColourButton.addEventListener("click", () => {
  if (state.colourBoxes.length >= maxColours) {
    return;
  }
  addColour(defaultColour);
  publishSettings();
});

mountSliders();
bootstrap();
setStatus("offline");
connect();
