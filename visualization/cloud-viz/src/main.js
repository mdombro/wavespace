import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

const DEFAULT_DATA_ROOT = "/@fs/home/matt/projects/wavespace/filtered-data-bounce";
const DATA_ROOT = (import.meta.env.VITE_DATA_ROOT || DEFAULT_DATA_ROOT).replace(/\/$/, "");
const MAX_POINTS = 2000;
const LOAD_CONCURRENCY = 8;
const FRAME_STRIDE = 1;
const MAX_FRAMES = 50000;
const PLAYBACK_FPS = 30;
const VOLUME_RES = 64;
const BASE_DENSITY = 2.2;

const container = document.getElementById("app") ?? document.body;
document.body.style.margin = "0";

const loadingEl = document.createElement("div");
loadingEl.style.cssText = [
  "position:fixed",
  "inset:0",
  "display:flex",
  "align-items:center",
  "justify-content:center",
  "background:#050608",
  "color:#e7ecf2",
  "font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono', 'Courier New', monospace",
  "font-size:14px",
  "letter-spacing:0.04em",
  "z-index:10",
].join(";");
loadingEl.textContent = "Loading point data...";
document.body.appendChild(loadingEl);

const uiPanel = document.createElement("div");
uiPanel.style.cssText = [
  "position:fixed",
  "left:16px",
  "bottom:16px",
  "padding:12px 14px",
  "background:rgba(10,12,16,0.72)",
  "border:1px solid rgba(255,255,255,0.12)",
  "border-radius:10px",
  "color:#e7ecf2",
  "font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono', 'Courier New', monospace",
  "font-size:12px",
  "letter-spacing:0.02em",
  "display:flex",
  "flex-direction:column",
  "gap:10px",
  "backdrop-filter:blur(6px)",
  "z-index:5",
].join(";");

function makeSliderRow(labelText, min, max, value, step) {
  const row = document.createElement("label");
  row.style.cssText = "display:flex;flex-direction:column;gap:6px;";
  const label = document.createElement("div");
  label.textContent = labelText;
  const input = document.createElement("input");
  input.type = "range";
  input.min = String(min);
  input.max = String(max);
  input.step = String(step);
  input.value = String(value);
  input.style.width = "180px";
  const valueText = document.createElement("div");
  valueText.style.color = "#8fb1ff";
  valueText.textContent = String(value);
  row.appendChild(label);
  row.appendChild(input);
  row.appendChild(valueText);
  return { row, input, valueText };
}

const timelineControl = makeSliderRow("Playback position", 0, 1, 0, 1);
const speedControl = makeSliderRow("Playback speed", 0.1, 3.0, 1.0, 0.05);
const brightnessControl = makeSliderRow("Color brightness", 0.6, 2.5, 1.2, 0.05);
const curveControl = makeSliderRow("Color curve", 0.4, 2.5, 0.9, 0.05);
const thresholdControl = makeSliderRow("Amplitude threshold", 0.0, 0.08, 0.03, 0.005);
const sharpnessControl = makeSliderRow("Sharpness", 0.0, 1.0, 0.55, 0.01);
const opacityControl = makeSliderRow("Cloud opacity", 0.1, 1.5, 0.9, 0.05);
const solidnessControl = makeSliderRow("Solidness", 0.0, 2.0, 0.6, 0.05);
const playPauseRow = document.createElement("div");
playPauseRow.style.cssText = "display:flex;gap:8px;";
const playButton = document.createElement("button");
const pauseButton = document.createElement("button");
playButton.textContent = "Play";
pauseButton.textContent = "Pause";
const buttonStyle = [
  "appearance:none",
  "border:1px solid rgba(255,255,255,0.18)",
  "background:rgba(15,18,24,0.9)",
  "color:#e7ecf2",
  "border-radius:6px",
  "padding:6px 10px",
  "cursor:pointer",
  "font:inherit",
].join(";");
playButton.style.cssText = buttonStyle;
pauseButton.style.cssText = buttonStyle;
playPauseRow.appendChild(playButton);
playPauseRow.appendChild(pauseButton);
const dataPickerRow = document.createElement("div");
dataPickerRow.style.cssText = "display:flex;align-items:center;gap:8px;flex-wrap:wrap;";
const dataButton = document.createElement("button");
dataButton.textContent = "Pick data folder";
dataButton.style.cssText = buttonStyle;
const dataLabel = document.createElement("div");
const defaultDataLabel = DATA_ROOT ? DATA_ROOT.replace(/^\/@fs/, "") : "not selected";
dataLabel.textContent = `Data: ${defaultDataLabel}`;
dataLabel.style.cssText = "opacity:0.75;max-width:220px;overflow:hidden;text-overflow:ellipsis;";
dataPickerRow.appendChild(dataButton);
dataPickerRow.appendChild(dataLabel);
timelineControl.valueText.textContent = "0/0";
speedControl.valueText.textContent = "1.00x";
brightnessControl.valueText.textContent = "1.20x";
curveControl.valueText.textContent = "0.90x";
thresholdControl.valueText.textContent = "0.030";
sharpnessControl.valueText.textContent = "0.55";
opacityControl.valueText.textContent = "0.90x";
solidnessControl.valueText.textContent = "0.60x";
timelineControl.input.style.width = "220px";
uiPanel.appendChild(playPauseRow);
uiPanel.appendChild(speedControl.row);
uiPanel.appendChild(brightnessControl.row);
uiPanel.appendChild(curveControl.row);
uiPanel.appendChild(thresholdControl.row);
uiPanel.appendChild(sharpnessControl.row);
uiPanel.appendChild(opacityControl.row);
uiPanel.appendChild(solidnessControl.row);
uiPanel.insertBefore(dataPickerRow, playPauseRow);
uiPanel.insertBefore(timelineControl.row, speedControl.row);
document.body.appendChild(uiPanel);

const folderInput = document.createElement("input");
folderInput.type = "file";
folderInput.style.display = "none";
folderInput.multiple = true;
folderInput.webkitdirectory = true;
folderInput.setAttribute("webkitdirectory", "");
folderInput.setAttribute("directory", "");
document.body.appendChild(folderInput);

const pointStatusEl = document.createElement("div");
pointStatusEl.style.cssText = [
  "position:fixed",
  "right:16px",
  "bottom:16px",
  "padding:10px 12px",
  "background:rgba(10,12,16,0.72)",
  "border:1px solid rgba(255,255,255,0.12)",
  "border-radius:10px",
  "color:#e7ecf2",
  "font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono', 'Courier New', monospace",
  "font-size:12px",
  "letter-spacing:0.02em",
  "backdrop-filter:blur(6px)",
  "z-index:5",
].join(";");
pointStatusEl.textContent = "Points loaded: 0";
document.body.appendChild(pointStatusEl);

function updatePointStatus(count, totalLabel) {
  if (totalLabel) {
    pointStatusEl.textContent = `Points loaded: ${count}/${totalLabel}`;
    return;
  }
  pointStatusEl.textContent = `Points loaded: ${count}`;
}

function setLoading(text) {
  loadingEl.textContent = text;
}

function clearLoading() {
  if (loadingEl.parentNode) {
    loadingEl.parentNode.removeChild(loadingEl);
  }
}

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

// --- renderer / scene / camera ---
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
container.appendChild(renderer.domElement);

if (!renderer.capabilities.isWebGL2) {
  document.body.innerHTML =
    "<pre>WebGL2 is required for 3D textures (sampler3D). Your browser/GPU is running WebGL1.</pre>";
  throw new Error("WebGL2 required");
}

renderer.debug.checkShaderErrors = true;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x05070a);
const camera = new THREE.PerspectiveCamera(
  55,
  window.innerWidth / window.innerHeight,
  0.1,
  100
);
camera.position.set(1.6, 1.2, 1.6);
camera.lookAt(0, 0, 0);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.enablePan = true;
controls.target.set(0, 0, 0);

const floatLinear = renderer.extensions.has("OES_texture_float_linear");
let playbackSpeed = parseFloat(speedControl.input.value);
let colorBrightness = parseFloat(brightnessControl.input.value);
let colorCurve = parseFloat(curveControl.input.value);
let ampThreshold = parseFloat(thresholdControl.input.value);
let sharpness = parseFloat(sharpnessControl.input.value);
let sharpnessGamma = 1.2;
let sharpnessSteps = 180;
let sharpnessThreshold = ampThreshold;
let opacityScale = parseFloat(opacityControl.input.value);
let solidness = parseFloat(solidnessControl.input.value);
let isScrubbing = false;
let playbackStartTime = 0;
let playbackStartFrame = 0;
let isPaused = false;
let pausedFrame = 0;
let dataDirectoryHandle = null;
let dataFileList = null;
let isLoadingDataset = false;
let loadToken = 0;
let animationStarted = false;

speedControl.input.addEventListener("input", () => {
  playbackSpeed = parseFloat(speedControl.input.value);
  speedControl.valueText.textContent = playbackSpeed.toFixed(2) + "x";
});

brightnessControl.input.addEventListener("input", () => {
  colorBrightness = parseFloat(brightnessControl.input.value);
  brightnessControl.valueText.textContent = colorBrightness.toFixed(2) + "x";
});

curveControl.input.addEventListener("input", () => {
  colorCurve = parseFloat(curveControl.input.value);
  curveControl.valueText.textContent = colorCurve.toFixed(2) + "x";
});

thresholdControl.input.addEventListener("input", () => {
  ampThreshold = parseFloat(thresholdControl.input.value);
  updateSharpnessMapping();
});

function updateSharpnessMapping() {
  sharpness = parseFloat(sharpnessControl.input.value);
  const t = clamp(sharpness, 0, 1);
  sharpnessGamma = 1.0 + (2.4 - 1.0) * t;
  sharpnessSteps = Math.round(160 + (320 - 160) * t);
  const thresholdBoost = 0.8 + (1.8 - 0.8) * t;
  sharpnessThreshold = clamp(ampThreshold * thresholdBoost, 0, 0.2);
  sharpnessControl.valueText.textContent = t.toFixed(2);
  thresholdControl.valueText.textContent =
    ampThreshold.toFixed(3) + " -> " + sharpnessThreshold.toFixed(3);
}

sharpnessControl.input.addEventListener("input", () => {
  updateSharpnessMapping();
});

opacityControl.input.addEventListener("input", () => {
  opacityScale = parseFloat(opacityControl.input.value);
  opacityControl.valueText.textContent = opacityScale.toFixed(2) + "x";
});

solidnessControl.input.addEventListener("input", () => {
  solidness = parseFloat(solidnessControl.input.value);
  solidnessControl.valueText.textContent = solidness.toFixed(2) + "x";
});

updateSharpnessMapping();
updatePlayPauseButtons();

function updateTimelineLabel(frameIndex) {
  if (!dataset) {
    timelineControl.valueText.textContent = "0/0";
    return;
  }
  timelineControl.valueText.textContent = `${frameIndex + 1}/${dataset.frameCount}`;
}

function currentFrameAt(time) {
  if (!dataset) {
    return 0;
  }
  const deltaFrames = Math.floor((time - playbackStartTime) * PLAYBACK_FPS * playbackSpeed);
  let frame = (playbackStartFrame + deltaFrames) % dataset.frameCount;
  if (frame < 0) {
    frame += dataset.frameCount;
  }
  return frame;
}

function updatePlayPauseButtons() {
  const active = "rgba(60,140,255,0.4)";
  const inactive = "rgba(15,18,24,0.9)";
  playButton.style.background = isPaused ? inactive : active;
  pauseButton.style.background = isPaused ? active : inactive;
}

async function setDataDirectory(handle) {
  dataDirectoryHandle = handle;
  dataFileList = null;
  dataLabel.textContent = `Data: ${handle.name || "selected folder"}`;
  await reloadDataset();
}

async function setDataFiles(files) {
  dataFileList = Array.from(files || []);
  dataDirectoryHandle = null;
  const root = dataFileList[0]?.webkitRelativePath?.split("/")?.[0];
  dataLabel.textContent = `Data: ${root || "selected folder"}`;
  await reloadDataset();
}

dataButton.addEventListener("click", async () => {
  try {
    if ("showDirectoryPicker" in window) {
      const handle = await window.showDirectoryPicker();
      await setDataDirectory(handle);
    } else {
      folderInput.click();
    }
  } catch (err) {
    if (err && err.name !== "AbortError") {
      console.error(err);
    }
  }
});

folderInput.addEventListener("change", () => {
  if (folderInput.files && folderInput.files.length) {
    void setDataFiles(folderInput.files);
    folderInput.value = "";
  }
});

playButton.addEventListener("click", () => {
  if (!dataset) {
    return;
  }
  if (isPaused) {
    isPaused = false;
    playbackStartFrame = pausedFrame;
    playbackStartTime = clock.getElapsedTime();
    updatePlayPauseButtons();
  }
});

pauseButton.addEventListener("click", () => {
  if (!dataset) {
    return;
  }
  if (!isPaused) {
    pausedFrame = currentFrameAt(clock.getElapsedTime());
    playbackStartFrame = pausedFrame;
    playbackStartTime = clock.getElapsedTime();
    isPaused = true;
    updatePlayPauseButtons();
  }
});

timelineControl.input.addEventListener("input", () => {
  if (!dataset) {
    return;
  }
  isScrubbing = true;
  const frame = Math.min(
    dataset.frameCount - 1,
    Math.max(0, Number.parseInt(timelineControl.input.value, 10) || 0)
  );
  playbackStartFrame = frame;
  playbackStartTime = clock.getElapsedTime();
  updateVolumeForFrame(frame);
  lastFrame = frame;
  if (isPaused) {
    pausedFrame = frame;
  }
  updateTimelineLabel(frame);
});

function endScrub() {
  if (!dataset) {
    return;
  }
  isScrubbing = false;
  const frame = Math.min(
    dataset.frameCount - 1,
    Math.max(0, Number.parseInt(timelineControl.input.value, 10) || 0)
  );
  playbackStartFrame = frame;
  playbackStartTime = clock.getElapsedTime();
  if (isPaused) {
    pausedFrame = frame;
  }
}

timelineControl.input.addEventListener("pointerdown", () => {
  isScrubbing = true;
});
timelineControl.input.addEventListener("pointerup", endScrub);
timelineControl.input.addEventListener("touchstart", () => {
  isScrubbing = true;
});
timelineControl.input.addEventListener("touchend", endScrub);

// --- NPZ + NPY loading helpers ---
const textDecoder = new TextDecoder("ascii");

async function inflateData(data) {
  if (typeof DecompressionStream === "undefined") {
    throw new Error("DecompressionStream not available in this browser.");
  }
  const stream = new Blob([data]).stream();
  try {
    const inflated = stream.pipeThrough(new DecompressionStream("deflate-raw"));
    return new Uint8Array(await new Response(inflated).arrayBuffer());
  } catch {
    const inflated = stream.pipeThrough(new DecompressionStream("deflate"));
    return new Uint8Array(await new Response(inflated).arrayBuffer());
  }
}

async function unzipNPZ(buffer) {
  const view = new DataView(buffer);
  const files = new Map();
  const cdEntries = readCentralDirectory(buffer);

  if (cdEntries) {
    for (const [name, entry] of cdEntries.entries()) {
      const data = await readZipEntry(buffer, entry, name);
      files.set(name, data);
    }
    return files;
  }

  let offset = 0;
  while (offset + 30 <= view.byteLength) {
    const signature = view.getUint32(offset, true);
    if (signature !== 0x04034b50) {
      break;
    }
    const method = view.getUint16(offset + 8, true);
    const compSize = view.getUint32(offset + 18, true);
    const uncompSize = view.getUint32(offset + 22, true);
    const nameLen = view.getUint16(offset + 26, true);
    const extraLen = view.getUint16(offset + 28, true);
    const nameStart = offset + 30;
    if (nameStart + nameLen + extraLen > view.byteLength) {
      throw new Error("Corrupt NPZ header (name/extra out of bounds).");
    }
    const name = textDecoder.decode(new Uint8Array(buffer, nameStart, nameLen));
    const dataStart = nameStart + nameLen + extraLen;
    if (dataStart + compSize > view.byteLength) {
      throw new Error(`Corrupt NPZ entry size for ${name}.`);
    }
    const compData = new Uint8Array(buffer, dataStart, compSize);

    let data;
    if (method === 0) {
      data = compData.slice();
    } else if (method === 8) {
      data = await inflateData(compData);
    } else {
      throw new Error(`Unsupported ZIP compression method: ${method}`);
    }
    if (uncompSize && data.length !== uncompSize) {
      data = data.slice(0, uncompSize);
    }
    files.set(name, data);
    offset = dataStart + compSize;
  }

  return files;
}

function findEndOfCentralDirectory(view) {
  const sig = 0x06054b50;
  const minOffset = Math.max(0, view.byteLength - 22 - 0xffff);
  for (let i = view.byteLength - 22; i >= minOffset; i--) {
    if (view.getUint32(i, true) === sig) {
      return i;
    }
  }
  return -1;
}

function readZip64Extra(view, offset, length, needs) {
  if (typeof view.getBigUint64 !== "function") {
    return { compSize: null, uncompSize: null, localOffset: null };
  }
  let compSize = null;
  let uncompSize = null;
  let localOffset = null;
  let cursor = offset;
  while (cursor + 4 <= offset + length) {
    const headerId = view.getUint16(cursor, true);
    const dataSize = view.getUint16(cursor + 2, true);
    cursor += 4;
    if (headerId === 0x0001) {
      let extraCursor = cursor;
      if (needs.uncomp) {
        uncompSize = Number(view.getBigUint64(extraCursor, true));
        extraCursor += 8;
      }
      if (needs.comp) {
        compSize = Number(view.getBigUint64(extraCursor, true));
        extraCursor += 8;
      }
      if (needs.offset) {
        localOffset = Number(view.getBigUint64(extraCursor, true));
      }
      break;
    }
    cursor += dataSize;
  }
  return { compSize, uncompSize, localOffset };
}

function readCentralDirectory(buffer) {
  const view = new DataView(buffer);
  const hasBigInt = typeof view.getBigUint64 === "function";
  const eocdOffset = findEndOfCentralDirectory(view);
  if (eocdOffset < 0) {
    return null;
  }
  const totalEntries = view.getUint16(eocdOffset + 10, true);
  const cdSize = view.getUint32(eocdOffset + 12, true);
  const cdOffset = view.getUint32(eocdOffset + 16, true);
  if (cdOffset + cdSize > view.byteLength) {
    return null;
  }

  const entries = new Map();
  let offset = cdOffset;
  for (let i = 0; i < totalEntries; i++) {
    if (view.getUint32(offset, true) !== 0x02014b50) {
      break;
    }
    const method = view.getUint16(offset + 10, true);
    let compSize = view.getUint32(offset + 20, true);
    let uncompSize = view.getUint32(offset + 24, true);
    const nameLen = view.getUint16(offset + 28, true);
    const extraLen = view.getUint16(offset + 30, true);
    const commentLen = view.getUint16(offset + 32, true);
    let localOffset = view.getUint32(offset + 42, true);
    const nameStart = offset + 46;
    const name = textDecoder.decode(new Uint8Array(buffer, nameStart, nameLen));

    const needsZip64 =
      compSize === 0xffffffff || uncompSize === 0xffffffff || localOffset === 0xffffffff;
    if (needsZip64) {
      if (!hasBigInt) {
        return null;
      }
      const extraStart = nameStart + nameLen;
      const zip64 = readZip64Extra(view, extraStart, extraLen, {
        comp: compSize === 0xffffffff,
        uncomp: uncompSize === 0xffffffff,
        offset: localOffset === 0xffffffff,
      });
      if (zip64.compSize !== null) {
        compSize = zip64.compSize;
      }
      if (zip64.uncompSize !== null) {
        uncompSize = zip64.uncompSize;
      }
      if (zip64.localOffset !== null) {
        localOffset = zip64.localOffset;
      }
    }

    entries.set(name, {
      method,
      compSize,
      uncompSize,
      localOffset,
    });
    offset = nameStart + nameLen + extraLen + commentLen;
  }
  return entries.size ? entries : null;
}

async function readZipEntry(buffer, entry, name) {
  const view = new DataView(buffer);
  const localOffset = entry.localOffset;
  if (localOffset + 30 > view.byteLength) {
    throw new Error(`Invalid local header offset for ${name}.`);
  }
  if (view.getUint32(localOffset, true) !== 0x04034b50) {
    throw new Error(`Invalid local header signature for ${name}.`);
  }
  const nameLen = view.getUint16(localOffset + 26, true);
  const extraLen = view.getUint16(localOffset + 28, true);
  const dataStart = localOffset + 30 + nameLen + extraLen;
  if (dataStart + entry.compSize > view.byteLength) {
    throw new Error(`Corrupt NPZ entry size for ${name}.`);
  }
  const compData = new Uint8Array(buffer, dataStart, entry.compSize);

  let data;
  if (entry.method === 0) {
    data = compData.slice();
  } else if (entry.method === 8) {
    data = await inflateData(compData);
  } else {
    throw new Error(`Unsupported ZIP compression method: ${entry.method}`);
  }
  if (entry.uncompSize && data.length !== entry.uncompSize) {
    data = data.slice(0, entry.uncompSize);
  }
  return data;
}

function parseNPY(bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const magic =
    String.fromCharCode(...bytes.slice(0, 6)) === "\x93NUMPY"
      ? "\x93NUMPY"
      : null;
  if (!magic) {
    throw new Error("Invalid NPY header.");
  }

  const major = view.getUint8(6);
  let headerLen = 0;
  let offset = 8;
  if (major === 1) {
    headerLen = view.getUint16(offset, true);
    offset += 2;
  } else if (major === 2) {
    headerLen = view.getUint32(offset, true);
    offset += 4;
  } else {
    throw new Error(`Unsupported NPY version: ${major}`);
  }

  const headerText = textDecoder.decode(bytes.slice(offset, offset + headerLen));
  const descrMatch = /'descr':\s*'([^']+)'/.exec(headerText);
  const shapeMatch = /'shape':\s*\(([^)]*)\)/.exec(headerText);

  if (!descrMatch || !shapeMatch) {
    throw new Error("Failed to parse NPY header.");
  }

  const descr = descrMatch[1];
  const shape = shapeMatch[1]
    .split(",")
    .map((part) => part.trim())
    .filter(Boolean)
    .map((part) => Number(part));

  const count = shape.reduce((acc, val) => acc * val, 1) || 1;
  const typeCode = descr[1];
  const bytesPer = Number(descr.slice(2));
  const littleEndian = descr[0] !== ">";
  if (!littleEndian) {
    throw new Error(`Unsupported big-endian dtype: ${descr}`);
  }

  const dtypeKey = `${typeCode}${bytesPer}`;
  const typeMap = {
    f4: Float32Array,
    f8: Float64Array,
    i4: Int32Array,
    u4: Uint32Array,
    i2: Int16Array,
    u2: Uint16Array,
    i1: Int8Array,
    u1: Uint8Array,
  };
  const ArrayType = typeMap[dtypeKey];
  if (!ArrayType) {
    throw new Error(`Unsupported dtype: ${descr}`);
  }

  const dataOffset = offset + headerLen;
  const byteLength = count * bytesPer;
  const dataBuffer = bytes.buffer.slice(
    bytes.byteOffset + dataOffset,
    bytes.byteOffset + dataOffset + byteLength
  );

  return {
    data: new ArrayType(dataBuffer),
    shape,
  };
}

function decimateSeries(series, stride, maxFrames) {
  const step = Math.max(1, stride);
  const count = Math.min(Math.ceil(series.length / step), maxFrames);
  const out = new Float32Array(count);
  let maxAbs = 0;
  for (let i = 0; i < count; i++) {
    const value = series[i * step];
    out[i] = value;
    const absVal = Math.abs(value);
    if (absVal > maxAbs) {
      maxAbs = absVal;
    }
  }
  return { series: out, maxAbs, count };
}

const POINT_FILE_PATTERN = /^point_(\d+)\.npz$/;

function parsePointIndex(name) {
  const match = POINT_FILE_PATTERN.exec(name);
  return match ? Number(match[1]) : null;
}

async function loadPointFromBuffer(buffer, sourceLabel, allowMissing) {
  const sigView = new DataView(buffer);
  const sig = sigView.byteLength >= 4 ? sigView.getUint32(0, true) : 0;
  if (sig !== 0x04034b50) {
    if (allowMissing) {
      return null;
    }
    throw new Error(`File is not a valid NPZ: ${sourceLabel}`);
  }
  const files = await unzipNPZ(buffer);
  const xyzBytes = files.get("xyz.npy");
  const valBytes = files.get("val.npy");
  if (!xyzBytes || !valBytes) {
    throw new Error(`Missing xyz/val arrays in ${sourceLabel}`);
  }
  const xyzParsed = parseNPY(xyzBytes);
  const valParsed = parseNPY(valBytes);
  const xyzData =
    xyzParsed.data instanceof Float32Array
      ? xyzParsed.data
      : Float32Array.from(xyzParsed.data);
  const valData =
    valParsed.data instanceof Float32Array
      ? valParsed.data
      : Float32Array.from(valParsed.data);

  return {
    xyz: [xyzData[0], xyzData[1], xyzData[2]],
    val: valData,
  };
}

async function loadPointFromEntry(entry) {
  const file = entry.file ?? (await entry.handle.getFile());
  const buffer = await file.arrayBuffer();
  return loadPointFromBuffer(buffer, entry.name, false);
}

async function listPointFilesFromDirectory(handle) {
  const entries = [];
  for await (const entry of handle.values()) {
    if (entry.kind !== "file") {
      continue;
    }
    const index = parsePointIndex(entry.name);
    if (index === null) {
      continue;
    }
    entries.push({ name: entry.name, index, handle: entry });
  }
  entries.sort((a, b) => a.index - b.index);
  return entries;
}

function listPointFilesFromFileList(files) {
  const entries = [];
  for (const file of files) {
    const index = parsePointIndex(file.name);
    if (index === null) {
      continue;
    }
    entries.push({ name: file.name, index, file });
  }
  entries.sort((a, b) => a.index - b.index);
  return entries;
}

async function getPointFileEntries() {
  if (dataDirectoryHandle) {
    return listPointFilesFromDirectory(dataDirectoryHandle);
  }
  if (dataFileList) {
    return listPointFilesFromFileList(dataFileList);
  }
  return null;
}

async function loadPointFile(index) {
  const id = String(index).padStart(5, "0");
  const url = `${DATA_ROOT}/point_${id}.npz`;
  const response = await fetch(url);
  if (!response.ok) {
    if (response.status === 404) {
      return null;
    }
    throw new Error(`Failed to fetch ${url}: ${response.status}`);
  }
  const buffer = await response.arrayBuffer();
  return loadPointFromBuffer(buffer, url, true);
}

async function loadPointDataset() {
  const points = [];
  const values = [];
  const xs = new Set();
  const ys = new Set();
  const zs = new Set();
  const key = (v) => Number(v.toFixed(6));
  let loadedCount = 0;
  let maxAbs = 0;
  let minFrameCount = Infinity;

  const entries = await getPointFileEntries();
  if ((dataDirectoryHandle || dataFileList) && (!entries || entries.length === 0)) {
    throw new Error("No point_*.npz files found in the selected folder.");
  }
  const hasEntries = Array.isArray(entries) && entries.length > 0;
  const limit = hasEntries ? Math.min(entries.length, MAX_POINTS) : MAX_POINTS;
  let nextIndex = 0;
  let stop = false;
  const totalLabel = hasEntries
    ? String(limit)
    : MAX_POINTS >= 1000
      ? `${MAX_POINTS}+`
      : String(MAX_POINTS);

  async function worker() {
    while (!stop && nextIndex < limit) {
      const idx = nextIndex++;
      const point = hasEntries
        ? await loadPointFromEntry(entries[idx])
        : await loadPointFile(idx);
      if (!point) {
        if (!hasEntries) {
          stop = true;
          return;
        }
        continue;
      }
      const xyz = [key(point.xyz[0]), key(point.xyz[1]), key(point.xyz[2])];
      xs.add(xyz[0]);
      ys.add(xyz[1]);
      zs.add(xyz[2]);
      const decimated = decimateSeries(point.val, FRAME_STRIDE, MAX_FRAMES);
      values[idx] = decimated.series;
      points[idx] = xyz;
      maxAbs = Math.max(maxAbs, decimated.maxAbs);
      minFrameCount = Math.min(minFrameCount, decimated.count);
      loadedCount += 1;
      if (loadedCount % 25 === 0) {
        updatePointStatus(loadedCount, totalLabel);
      }
      if (idx % 25 === 0) {
        const progress = Math.min(idx + 1, limit);
        setLoading(`Loading point data... ${progress}/${totalLabel}`);
      }
    }
  }

  const workers = Array.from({ length: LOAD_CONCURRENCY }, worker);
  await Promise.all(workers);

  const filteredPoints = points.filter(Boolean);
  const filteredValues = values.filter(Boolean);

  if (!filteredPoints.length) {
    throw new Error("No point files found.");
  }

  updatePointStatus(filteredPoints.length, hasEntries ? totalLabel : undefined);

  const xVals = Array.from(xs).sort((a, b) => a - b);
  const yVals = Array.from(ys).sort((a, b) => a - b);
  const zVals = Array.from(zs).sort((a, b) => a - b);

  const xIndex = new Map(xVals.map((v, i) => [v, i]));
  const yIndex = new Map(yVals.map((v, i) => [v, i]));
  const zIndex = new Map(zVals.map((v, i) => [v, i]));

  const gridX = xVals.length;
  const gridY = yVals.length;
  const gridZ = zVals.length;
  const gridSize = gridX * gridY * gridZ;
  const pointToGrid = new Int32Array(filteredPoints.length);

  for (let i = 0; i < filteredPoints.length; i++) {
    const [x, y, z] = filteredPoints[i];
    const xi = xIndex.get(x);
    const yi = yIndex.get(y);
    const zi = zIndex.get(z);
    if (xi === undefined || yi === undefined || zi === undefined) {
      throw new Error("Point coordinate does not match grid axis values.");
    }
    pointToGrid[i] = xi + gridX * (yi + gridY * zi);
  }

  return {
    points: filteredPoints,
    values: filteredValues,
    frameCount: Math.max(1, minFrameCount),
    maxAbs: maxAbs > 0 ? maxAbs : 1,
    valueScale: 1 / (maxAbs > 0 ? maxAbs : 1),
    gridDims: { x: gridX, y: gridY, z: gridZ, size: gridSize },
    pointToGrid,
  };
}

function buildAxisSampler(srcLen, dstLen) {
  const i0 = new Int32Array(dstLen);
  const i1 = new Int32Array(dstLen);
  const t = new Float32Array(dstLen);
  const denom = Math.max(1, dstLen - 1);
  const maxSrc = Math.max(1, srcLen - 1);
  for (let i = 0; i < dstLen; i++) {
    const pos = (i / denom) * maxSrc;
    const lo = Math.floor(pos);
    const hi = Math.min(lo + 1, maxSrc);
    i0[i] = lo;
    i1[i] = hi;
    t[i] = pos - lo;
  }
  return { i0, i1, t };
}

function resampleTrilinear(src, srcX, srcY, srcZ, dst, dstX, dstY, dstZ, sx, sy, sz) {
  let di = 0;
  for (let z = 0; z < dstZ; z++) {
    const z0 = sz.i0[z];
    const z1 = sz.i1[z];
    const tz = sz.t[z];
    const z0Offset = z0 * srcX * srcY;
    const z1Offset = z1 * srcX * srcY;
    for (let y = 0; y < dstY; y++) {
      const y0 = sy.i0[y];
      const y1 = sy.i1[y];
      const ty = sy.t[y];
      const y0Offset = y0 * srcX;
      const y1Offset = y1 * srcX;
      for (let x = 0; x < dstX; x++) {
        const x0 = sx.i0[x];
        const x1 = sx.i1[x];
        const tx = sx.t[x];

        const c000 = src[z0Offset + y0Offset + x0];
        const c100 = src[z0Offset + y0Offset + x1];
        const c010 = src[z0Offset + y1Offset + x0];
        const c110 = src[z0Offset + y1Offset + x1];
        const c001 = src[z1Offset + y0Offset + x0];
        const c101 = src[z1Offset + y0Offset + x1];
        const c011 = src[z1Offset + y1Offset + x0];
        const c111 = src[z1Offset + y1Offset + x1];

        const c00 = c000 + (c100 - c000) * tx;
        const c10 = c010 + (c110 - c010) * tx;
        const c01 = c001 + (c101 - c001) * tx;
        const c11 = c011 + (c111 - c011) * tx;
        const c0 = c00 + (c10 - c00) * ty;
        const c1 = c01 + (c11 - c01) * ty;
        dst[di++] = c0 + (c1 - c0) * tz;
      }
    }
  }
}

let dataset = null;
let volumeData = null;
let gridData = null;
let tex3d = null;
let material = null;
let box = null;
let axisSampler = null;
let lastFrame = -1;

function buildVolume() {
  if (box) {
    scene.remove(box);
    if (box.geometry) {
      box.geometry.dispose();
    }
  }
  if (material) {
    material.dispose();
  }
  if (tex3d) {
    tex3d.dispose();
  }

  const size = VOLUME_RES * VOLUME_RES * VOLUME_RES;
  volumeData = new Float32Array(size);
  tex3d = new THREE.Data3DTexture(volumeData, VOLUME_RES, VOLUME_RES, VOLUME_RES);
  tex3d.format = THREE.RedFormat;
  tex3d.type = THREE.FloatType;
  tex3d.minFilter = floatLinear ? THREE.LinearFilter : THREE.NearestFilter;
  tex3d.magFilter = floatLinear ? THREE.LinearFilter : THREE.NearestFilter;
  tex3d.unpackAlignment = 1;
  tex3d.needsUpdate = true;

  const boxGeo = new THREE.BoxGeometry(2, 2, 2);

  material = new THREE.ShaderMaterial({
    transparent: true,
    side: THREE.BackSide,
    uniforms: {
      uTex: { value: tex3d },
      uTime: { value: 0 },
      uSteps: { value: sharpnessSteps },
      uDensity: { value: BASE_DENSITY },
      uGamma: { value: sharpnessGamma },
      uThreshold: { value: sharpnessThreshold },
      uBrightness: { value: colorBrightness },
      uCurve: { value: colorCurve },
      uOpacity: { value: opacityScale },
      uCamPos: { value: new THREE.Vector3() },
    },
    vertexShader: `
      varying vec3 vPos;
      void main() {
        vPos = position;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: `
      precision highp float;
      precision highp sampler3D;

      uniform sampler3D uTex;
      uniform float uTime;
      uniform float uSteps;
      uniform float uDensity;
      uniform float uGamma;
      uniform float uThreshold;
      uniform float uBrightness;
      uniform float uCurve;
      uniform float uOpacity;
      uniform vec3 uCamPos;

      varying vec3 vPos;

      vec3 toTex(vec3 p) {
        return p * 0.5 + 0.5;
      }

      vec3 pressureColor(float p) {
        vec3 warm = vec3(1.0, 0.35, 0.15);
        vec3 cool = vec3(0.15, 0.45, 1.0);
        return mix(cool, warm, step(0.0, p));
      }

      void main() {
        vec3 rayOrigin = vPos;
        vec3 rayDir = normalize(uCamPos - vPos);

        float tMin = 0.0;
        float tMax = 3.0;
        float dt = (tMax - tMin) / uSteps;

        vec3 accumColor = vec3(0.0);
        float accumAlpha = 0.0;

        float t = tMin;
        for (int i = 0; i < 512; i++) {
          if (float(i) >= uSteps) break;

          vec3 p = rayOrigin + t * rayDir;
          if (any(greaterThan(abs(p), vec3(1.0)))) {
            t += dt;
            continue;
          }

          float pressure = texture(uTex, toTex(p)).r;
          float mag = abs(pressure);
          if (mag > uThreshold) {
            float shaped = pow(mag, uCurve);
            float dens = pow(shaped, uGamma) * uDensity;
            float a = (1.0 - exp(-dens * dt)) * uOpacity;
            a = clamp(a, 0.0, 1.0);
            vec3 col = pressureColor(pressure);
            vec3 sampleColor = col * dens * uBrightness;
            sampleColor *= a;
            accumColor += (1.0 - accumAlpha) * sampleColor;
            accumAlpha += (1.0 - accumAlpha) * a;
            if (accumAlpha > 0.98) break;
          }
          t += dt;
        }

        gl_FragColor = vec4(accumColor, accumAlpha);
      }
    `,
  });

  box = new THREE.Mesh(boxGeo, material);
  scene.add(box);
}

function updateVolumeForFrame(frameIndex) {
  if (!dataset || !volumeData || !gridData) {
    return;
  }

  const { values, pointToGrid, gridDims, valueScale } = dataset;
  const gridSize = gridDims.size;
  const gridFull = values.length === gridSize;

  if (!gridFull) {
    gridData.fill(0);
  }

  for (let i = 0; i < values.length; i++) {
    const series = values[i];
    gridData[pointToGrid[i]] = series[frameIndex] * valueScale;
  }

  if (gridDims.x === VOLUME_RES && gridDims.y === VOLUME_RES && gridDims.z === VOLUME_RES) {
    volumeData.set(gridData);
  } else {
    resampleTrilinear(
      gridData,
      gridDims.x,
      gridDims.y,
      gridDims.z,
      volumeData,
      VOLUME_RES,
      VOLUME_RES,
      VOLUME_RES,
      axisSampler.x,
      axisSampler.y,
      axisSampler.z
    );
  }

  tex3d.needsUpdate = true;
}

// --- animation loop ---
const clock = new THREE.Clock();

function animate() {
  requestAnimationFrame(animate);

  if (!dataset) {
    renderer.render(scene, camera);
    return;
  }

  const t = clock.getElapsedTime();
  const frame = isPaused ? pausedFrame : currentFrameAt(t);

  if (frame !== lastFrame) {
    updateVolumeForFrame(frame);
    lastFrame = frame;
  }

  if (!isScrubbing) {
    timelineControl.input.value = String(frame);
    updateTimelineLabel(frame);
  }

  material.uniforms.uTime.value = t;
  material.uniforms.uBrightness.value = colorBrightness;
  material.uniforms.uCurve.value = colorCurve;
  material.uniforms.uThreshold.value = sharpnessThreshold;
  material.uniforms.uGamma.value = sharpnessGamma;
  material.uniforms.uSteps.value = sharpnessSteps;
  const densityBoost = 1 + solidness * 1.5;
  const opacityBoost = 1 + solidness * 0.8;
  material.uniforms.uDensity.value = BASE_DENSITY * densityBoost;
  material.uniforms.uOpacity.value = clamp(opacityScale * opacityBoost, 0.02, 3.0);
  material.uniforms.uCamPos.value.copy(camera.position);
  controls.update();

  renderer.render(scene, camera);
}

async function reloadDataset() {
  const token = ++loadToken;
  isLoadingDataset = true;
  setLoading("Loading point data...");
  updatePointStatus(0);
  dataset = null;
  lastFrame = -1;

  try {
    const loaded = await loadPointDataset();
    if (token !== loadToken) {
      return;
    }
    dataset = loaded;
    timelineControl.input.max = String(Math.max(0, dataset.frameCount - 1));
    timelineControl.input.value = "0";
    updateTimelineLabel(0);
    playbackStartTime = clock.getElapsedTime();
    playbackStartFrame = 0;
    pausedFrame = 0;
    isPaused = false;
    updatePlayPauseButtons();
    gridData = new Float32Array(dataset.gridDims.size);
    axisSampler = {
      x: buildAxisSampler(dataset.gridDims.x, VOLUME_RES),
      y: buildAxisSampler(dataset.gridDims.y, VOLUME_RES),
      z: buildAxisSampler(dataset.gridDims.z, VOLUME_RES),
    };
    buildVolume();
    updateVolumeForFrame(0);
    clearLoading();
    clock.start();
    if (!animationStarted) {
      animationStarted = true;
      animate();
    }
  } catch (err) {
    console.error(err);
    if (token === loadToken) {
      setLoading("Failed to load NPZ data. See console for details.");
    }
  } finally {
    if (token === loadToken) {
      isLoadingDataset = false;
    }
  }
}

async function init() {
  await reloadDataset();
}

init();

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
});
