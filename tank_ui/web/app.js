const el = (id) => document.getElementById(id);

function fmtPct(v) {
  if (v == null || Number.isNaN(Number(v))) return "—";
  return Math.round(Number(v)) + "%";
}
function setRing(id, pct, hotAt, warnAt) {
  const n = el(id);
  if (!n) return;
  const p = Math.max(0, Math.min(100, Number(pct) || 0));
  n.style.setProperty("--p", String(p));
  n.classList.toggle("hot", p >= (hotAt || 90));
  n.classList.toggle("warn", p >= (warnAt || 75) && p < (hotAt || 90));
}

async function pollSys() {
  try {
    const j = await (await fetch("/api/sys")).json();
    setRing("ring-cpu", j.cpu_pct, 90, 75);
    el("sys-cpu").textContent = fmtPct(j.cpu_pct);
    setRing("ring-gpu", j.gpu_pct, 90, 75);
    el("sys-gpu").textContent = fmtPct(j.gpu_pct);
    setRing("ring-ram", j.ram_pct, 92, 80);
    el("sys-ram").textContent = fmtPct(j.ram_pct);
    const t = j.temp_c;
    const tPct = t == null ? 0 : Math.max(0, Math.min(100, ((Number(t) - 25) / 70) * 100));
    const tr = el("ring-tmp");
    tr.style.setProperty("--p", String(tPct));
    tr.classList.toggle("hot", t != null && t >= 80);
    tr.classList.toggle("warn", t != null && t >= 70 && t < 80);
    el("sys-tmp").textContent = t == null ? "—" : Math.round(t) + "°";
    const bits = [];
    if (j.temp_cpu_c != null) bits.push("cpu " + j.temp_cpu_c.toFixed(0) + "°");
    if (j.temp_gpu_c != null) bits.push("gpu " + j.temp_gpu_c.toFixed(0) + "°");
    if (j.ram_total_gb) bits.push(j.ram_used_gb.toFixed(1) + "/" + j.ram_total_gb.toFixed(1) + " Go");
    el("sys-hud").title = bits.join(" · ") || "sys";
  } catch (e) {
    /* ignore */
  }
}
setInterval(pollSys, 1000);
pollSys();

/* ---- tabs ---- */
document.querySelectorAll(".tab").forEach((btn) => {
  btn.addEventListener("click", () => {
    if (btn.disabled) return;
    document.querySelectorAll(".tab").forEach((b) => b.classList.remove("on"));
    document.querySelectorAll(".panel").forEach((p) => p.classList.remove("on"));
    btn.classList.add("on");
    el("tab-" + btn.dataset.tab).classList.add("on");
    if (btn.dataset.tab === "track") refreshStream();
    if (btn.dataset.tab === "odom") refreshRsStream();
  });
});

/* ---- ROS stacks ---- */
let lastStacks = [];

async function pollStacks() {
  try {
    const j = await (await fetch("/api/stacks")).json();
    lastStacks = j.stacks || [];
    renderStacks(j.by_category || {});
    const sel = el("log-stack");
    const cur = sel.value;
    sel.innerHTML = lastStacks
      .map((s) => `<option value="${s.name}">${s.label}</option>`)
      .join("");
    if (cur) sel.value = cur;
  } catch (e) {
    /* ignore */
  }
}

function renderStacks(byCat) {
  const grid = el("stack-grid");
  const order = ["pantilt", "base", "viz", "foxglove", "vlm", "track"];
  const cats = [...order, ...Object.keys(byCat).filter((c) => !order.includes(c))];
  let html = "";
  for (const cat of cats) {
    const list = byCat[cat];
    if (!list || !list.length) continue;
    for (const s of list) {
      const nodes = (s.nodes_present || [])
        .map((n) => `<span style="color:#3a7">${n}</span>`)
        .concat((s.nodes_missing || []).map((n) => `<span style="color:#b3261e">${n}</span>`))
        .join("");
      html += `<div class="stack">
        <div class="stack-head">
          <span class="dot ${s.color}" title="${s.color}"></span>
          <strong>${s.label}</strong>
        </div>
        <div class="nodes">${nodes || "—"}</div>
        <div class="btns">
          <button class="ok" onclick="stackStart('${s.name}')">Start</button>
          <button class="danger" onclick="stackStop('${s.name}')">Stop</button>
          <button class="sec" onclick="showLog('${s.name}')">Logs</button>
        </div>
      </div>`;
    }
  }
  const next = html || "<p class='hint'>no stacks configured</p>";
  if (grid.dataset.hash === next) return;
  grid.dataset.hash = next;
  grid.innerHTML = next;
}

async function stackStart(name) {
  const r = await fetch("/api/stack/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ stack: name }),
  });
  flash(await r.json());
  pollStacks();
}
async function stackStop(name) {
  const r = await fetch("/api/stack/stop", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ stack: name }),
  });
  flash(await r.json());
  pollStacks();
}
function showLog(name) {
  el("log-stack").value = name;
  refreshLogs();
}
async function refreshLogs() {
  const stack = el("log-stack").value;
  if (!stack) return;
  const j = await (await fetch(`/api/logs?stack=${encodeURIComponent(stack)}&n=200`)).json();
  el("log-view").textContent = (j.lines || []).join("\n") || j.message || "—";
  el("log-view").scrollTop = el("log-view").scrollHeight;
}

function flash(j) {
  const s = el("pt-state");
  if (!s) return;
  const line = (j.ok ? "✓ " : "✗ ") + (j.message || JSON.stringify(j));
  s.textContent = line + "\n" + (s.dataset.base || "");
}

setInterval(pollStacks, 500);
pollStacks();

/* ---- Pantilt ---- */
let measured = { pan: 0, tilt: 0, mode: "?" };

["pan_dps", "tilt_dps"].forEach((id) => {
  el(id).addEventListener("input", () => {
    el(id + "_v").textContent = el(id).value;
  });
});

function zeroVel() {
  el("pan_dps").value = 0;
  el("tilt_dps").value = 0;
  el("pan_dps_v").textContent = "0";
  el("tilt_dps_v").textContent = "0";
}
function sendAng() {
  fetch("/api/ang", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      pan_deg: Number(el("pan_deg").value),
      tilt_deg: Number(el("tilt_deg").value),
    }),
  });
}
function copyMeas() {
  el("pan_deg").value = measured.pan.toFixed(1);
  el("tilt_deg").value = measured.tilt.toFixed(1);
}
async function setMode(mode) {
  if (mode === "IDLE") zeroVel();
  const r = await fetch("/api/mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode }),
  });
  flash(await r.json());
}
async function callHome() {
  flash(await (await fetch("/api/home", { method: "POST" })).json());
}
async function callStop() {
  zeroVel();
  flash(await (await fetch("/api/stop", { method: "POST" })).json());
}
function applyMode(mode) {
  const m = (mode || "?").toUpperCase();
  const badge = el("mode_badge");
  badge.textContent = m;
  badge.className =
    "badge badge-" + (["IDLE", "HOMING", "RUNNING"].includes(m) ? m : "UNKNOWN");
  ["IDLE", "HOMING", "RUNNING"].forEach((x) => {
    const b = el("mode-" + x);
    if (b) b.classList.toggle("on", x === m);
  });
  const run = m === "RUNNING";
  el("pan_dps").disabled = !run;
  el("tilt_dps").disabled = !run;
}

setInterval(() => {
  fetch("/api/vel", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      pan_dps: Number(el("pan_dps").value),
      tilt_dps: Number(el("tilt_dps").value),
    }),
  });
}, 50);

setInterval(async () => {
  try {
    const j = await (await fetch("/api/state")).json();
    measured = j;
    applyMode(j.mode);
    const txt = `mode: ${j.mode}\npan:  ${j.pan.toFixed(2)}°\ntilt: ${j.tilt.toFixed(2)}°\njs:   ${j.connected ? "live" : "waiting"}`;
    el("pt-state").dataset.base = txt;
    el("pt-state").textContent = txt;
  } catch (e) {
    el("pt-state").textContent = "state poll failed";
  }
}, 200);

/* ---- Tracking ---- */
const CSI_TOPIC = "/csi_cam/image_raw";
const TRACK_TOPIC = "/vlm/track_annotated/compressed";
const CROP_TOPIC = "/vlm/track_crop/compressed";
let streamTopic = "";
let cropOn = false;
let pidFilled = false;
let lastTrackUp = false;
const PID_KEYS = ["kp_x", "kp_y", "kd_x", "kd_y", "ki_x", "ki_y", "deadzone_px", "max_dps"];

function desiredStream(j) {
  if (j.stream) return j.stream;
  const fb = j.feedback || {};
  const tracking = fb.active && (fb.phase === "TRACKING" || fb.phase === "FOUND");
  return tracking ? TRACK_TOPIC : CSI_TOPIC;
}

function applyStream(topic) {
  if (topic === streamTopic) return;
  streamTopic = topic;
  el("stream").src = `/ui/stream?topic=${encodeURIComponent(topic)}&t=${Date.now()}`;
  el("stream-src").textContent = topic === TRACK_TOPIC ? "tracker" : "csi";
}

function refreshStream() {
  applyStream(streamTopic || CSI_TOPIC);
}

const RS_TOPIC = "/camera/camera/color/image_raw";
function refreshRsStream() {
  const n = el("rs-stream");
  if (!n) return;
  n.src = `/ui/stream?topic=${encodeURIComponent(RS_TOPIC)}&t=${Date.now()}`;
}

function fmtOdom(j, label) {
  if (!j || !j.ok) return `${label}: —`;
  return [
    `${label}  x=${j.x.toFixed(3)} y=${j.y.toFixed(3)} yaw=${j.yaw_deg.toFixed(1)}°`,
    `     vx=${j.vx.toFixed(3)} vy=${j.vy.toFixed(3)} wz=${j.wz.toFixed(3)}  age=${j.age_s.toFixed(2)}s`,
  ].join("\n");
}

async function odomMode(mode) {
  await fetch("/api/odom/mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode }),
  });
}

async function odomZero() {
  await fetch("/api/odom/zero", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: "{}",
  });
}

setInterval(async () => {
  try {
    const j = await (await fetch("/api/odom")).json();
    const pre = el("odom-state");
    const src = el("odom-src");
    if (!pre) return;
    if (src) src.textContent = j.source ? `/odom ← ${j.source}` : "/odom";
    if (!j.ok) {
      pre.textContent = "no odom — start Base (wheels) and/or VO";
      return;
    }
    pre.textContent = [
      fmtOdom(j.wheel, "wheel"),
      fmtOdom(j.vo, "vo   "),
      fmtOdom(j.fused, "mux  "),
    ].join("\n");
  } catch (e) {
    /* ignore */
  }
}, 200);

function applyCrop(show) {
  const n = el("vlm-crop");
  if (!n) return;
  if (show) {
    n.classList.remove("hidden");
    if (!cropOn) {
      cropOn = true;
      n.src = `/ui/stream?topic=${encodeURIComponent(CROP_TOPIC)}&t=${Date.now()}`;
    }
  } else if (cropOn) {
    cropOn = false;
    n.src = "";
    n.classList.add("hidden");
  }
}

function readPid() {
  const d = {};
  for (const k of PID_KEYS) {
    d[k] = Number(el("pid-" + k).value);
  }
  return d;
}

function fillPid(pid) {
  if (!pid || pidFilled) return;
  for (const k of PID_KEYS) {
    if (pid[k] == null) continue;
    const n = el("pid-" + k);
    if (n) n.value = pid[k];
  }
  pidFilled = true;
}

let pidTimer = null;
function pushPid() {
  fetch("/api/track/pid", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(readPid()),
  });
}

async function pidSave() {
  const r = await fetch("/api/track/pid/save", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(readPid()),
  });
  const j = await r.json();
  el("track-fb").textContent = (j.ok ? "✓ " : "✗ ") + (j.message || "");
}

PID_KEYS.forEach((k) => {
  const n = el("pid-" + k);
  if (!n) return;
  n.addEventListener("change", pushPid);
  n.addEventListener("input", () => {
    clearTimeout(pidTimer);
    pidTimer = setTimeout(pushPid, 250);
  });
});

el("motion-enable").addEventListener("change", async () => {
  await fetch("/api/track/motion", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ enabled: el("motion-enable").checked }),
  });
});

el("sweep-enable").addEventListener("change", async () => {
  await fetch("/api/track/sweep", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ enabled: el("sweep-enable").checked }),
  });
});

async function trackStart() {
  const target = el("track-target").value.trim();
  if (!target) return;
  pushPid();
  const r = await fetch("/api/track/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      target,
      do_home: el("track-home").checked,
    }),
  });
  const j = await r.json();
  el("track-fb").textContent = (j.ok ? "✓ " : "✗ ") + (j.message || "");
}
async function trackCancel() {
  const j = await (await fetch("/api/track/cancel", { method: "POST" })).json();
  el("track-fb").textContent = (j.ok ? "✓ " : "✗ ") + (j.message || "");
}

function fmtPx(v) {
  if (v == null || Number.isNaN(Number(v))) return "—";
  const n = Number(v);
  return (n >= 0 ? "+" : "") + n.toFixed(0);
}

setInterval(async () => {
  try {
    const j = await (await fetch("/api/track/state")).json();
    const warn = el("track-warn");
    if (j.ready) warn.classList.add("hidden");
    else warn.classList.remove("hidden");
    if (el("motion-enable") !== document.activeElement) {
      el("motion-enable").checked = !!j.motion_enabled;
    }
    if (el("sweep-enable") !== document.activeElement) {
      el("sweep-enable").checked = !!j.sweep_enabled;
    }
    fillPid(j.pid);
    if (j.track_up && !lastTrackUp) {
      pushPid();
      fetch("/api/track/sweep", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ enabled: el("sweep-enable").checked }),
      });
      fetch("/api/track/motion", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ enabled: el("motion-enable").checked }),
      });
    }
    lastTrackUp = !!j.track_up;
    applyStream(desiredStream(j));
    applyCrop(!!(j.feedback && (j.feedback.phase === "TRACKING" || j.feedback.phase === "FOUND")));
    const fb = j.feedback || {};
    const hasBox = (fb.bbox_xyxy || []).some((v) => v);
    const dxEl = el("dx");
    const dyEl = el("dy");
    const live = fb.active && (fb.phase === "TRACKING" || fb.phase === "FOUND") && hasBox;
    dxEl.textContent = live ? fmtPx(fb.dx_px) : "—";
    dyEl.textContent = live ? fmtPx(fb.dy_px) : "—";
    dxEl.className = dyEl.className = live ? "on" : "off";
    el("track-fb").textContent = [
      `status: ${fb.status || "idle"}`,
      `target: ${fb.target || "—"}`,
      `phase:  ${fb.phase || "—"}`,
      `step:   ${fb.explore_step || "—"}`,
      `score:  ${(fb.score || 0).toFixed(3)}`,
      `dX/dY:  ${fmtPx(fb.dx_px)} / ${fmtPx(fb.dy_px)} px`,
      `pid:    pan=${(fb.pan_dps || 0).toFixed(1)} tilt=${(fb.tilt_dps || 0).toFixed(1)} dps`,
      `esp:    ${j.motion_enabled ? "on" : "gated"}`,
      fb.message ? `msg:    ${fb.message}` : "",
      `cam=${j.cam_ok} vlm=${j.vlm_up} track=${j.track_up}`,
    ]
      .filter(Boolean)
      .join("\n");
  } catch (e) {
    /* ignore */
  }
}, 300);
