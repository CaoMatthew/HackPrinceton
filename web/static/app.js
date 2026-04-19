const $ = (sel, root = document) => root.querySelector(sel);

const els = {
  form:           $("#plan-form"),
  task:           $("#task"),
  runSim:         $("#run-sim"),
  submitBtn:      $("#submit-btn"),
  startGui:       $("#start-gui"),
  killGui:        $("#kill-gui"),
  guiDot:         $("#gui-dot"),
  guiLabel:       $("#gui-status-label"),
  guiHint:        $("#gui-hint"),
  xray:           $("#xray-timeline"),
  empty:          $("#xray-empty"),
  sessionLog:     $("#session-log"),
  adapterUrl:     $("#adapter-url"),
  adapterHeaders: $("#adapter-headers"),
  adapterEnabled: $("#adapter-enabled"),
  saveAdapter:    $("#save-adapter"),
  copyTrace:      $("#copy-trace"),
  clearLog:       $("#clear-log"),
  lastJson:       null,
};

// ── Utility ───────────────────────────────────────────────────────────────────

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

// ── Session log ───────────────────────────────────────────────────────────────

function _addLog(html, cssClass) {
  const empty = els.sessionLog.querySelector(".session-empty");
  if (empty) empty.remove();
  const ts = new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" });
  const row = document.createElement("div");
  row.className = "session-entry" + (cssClass ? " " + cssClass : "");
  row.innerHTML = `<span class="ts">${ts}</span>${html}`;
  els.sessionLog.appendChild(row);
  if (els.sessionLog.children.length > 60)
    els.sessionLog.removeChild(els.sessionLog.firstChild);
  els.sessionLog.scrollTop = els.sessionLog.scrollHeight;
}

const logCmd     = (t) => _addLog(`<span class="log-label cmd-label">OPERATOR &gt;</span><span class="msg">${escapeHtml(t)}</span>`, "cmd");
const logWorking = (t) => _addLog(`<span class="log-label working-label">WORKING</span><span class="msg">${escapeHtml(t)}</span>`, "working");
const logReady   = (t) => _addLog(`<span class="log-label ready-label">READY</span><span class="msg">${escapeHtml(t)}</span>`, "ready");
const logError   = (t) => _addLog(`<span class="log-label error-label">ERROR</span><span class="msg">${escapeHtml(t)}</span>`, "error");
const logInfo    = (t) => _addLog(`<span class="msg muted">${escapeHtml(t)}</span>`, "info");

// ── X-ray: pure append-only (terminal style) ─────────────────────────────────
// Stages stack and stay — nothing is ever replaced or cleared mid-run.
// Only xrayClear() removes old output, called once at the start of each command.

function xrayClear() {
  els.empty.style.display = "none";
  els.xray.removeAttribute("hidden");
  els.xray.innerHTML = "";
}

function _buildStageDetail(st) {
  if (st.lines) {
    return `<ul class="action-list">${st.lines.map((l) => `<li>${escapeHtml(l)}</li>`).join("")}</ul>`;
  }
  if (st.exitCode !== undefined) {
    const badge = st.exitCode === 0
      ? `<span class="exit-ok">exit 0 // OK</span>`
      : `<span class="exit-err">exit ${st.exitCode} // FAILED</span>`;
    return `${badge}<pre class="xray-pre">${escapeHtml(st.detail || "")}</pre>`;
  }
  return `<pre class="xray-pre">${escapeHtml(st.detail || "")}</pre>`;
}

function xrayAppend(st) {
  const accent = st.accent || "blue";
  const tag    = accent === "red" ? "RED CHANNEL" : "BLUE CHANNEL";
  const sub    = st.subtitle ? `<p class="stage-sub">${escapeHtml(st.subtitle)}</p>` : "";
  const el     = document.createElement("article");
  el.className      = "stage";
  el.dataset.accent  = accent;
  el.dataset.stageId = st.id || "";
  el.innerHTML = `
    <div class="stage-head">
      <div><h3 class="stage-title">${escapeHtml(st.label)}</h3>${sub}</div>
      <span class="tag accent">${tag}</span>
    </div>
    <div class="stage-body" data-body>${_buildStageDetail(st)}</div>`;
  els.xray.appendChild(el);
  el.scrollIntoView({ behavior: "smooth", block: "end" });
  return el;
}

// Live-append text to an existing stage's <pre> (for sim stdout streaming).
// Creates the stage on first call; subsequent calls just add new lines.
function xrayAppendSimLine(line) {
  let el = els.xray.querySelector('[data-stage-id="sim"]');
  if (!el) {
    el = xrayAppend({ id: "sim", label: "SUBPROCESS // PYBULLET",
      subtitle: "Live stdout from sim_persistent.py", detail: "", accent: "blue" });
  }
  const pre = el.querySelector(".xray-pre");
  if (pre) {
    pre.textContent += (pre.textContent ? "\n" : "") + line;
    el.scrollIntoView({ behavior: "smooth", block: "end" });
  }
}

// Finalise the sim stage with an exit-code badge.
function xrayFinalSim(exitCode, stdout) {
  let el = els.xray.querySelector('[data-stage-id="sim"]');
  if (!el) {
    el = xrayAppend({ id: "sim", label: "SUBPROCESS // PYBULLET",
      subtitle: "", detail: stdout || "", accent: exitCode === 0 ? "blue" : "red", exitCode });
    return;
  }
  el.dataset.accent = exitCode === 0 ? "blue" : "red";
  const body = el.querySelector("[data-body]");
  if (body) {
    const badge = exitCode === 0
      ? `<span class="exit-ok">exit 0 // OK</span>`
      : `<span class="exit-err">exit ${exitCode} // FAILED</span>`;
    const existing = el.querySelector(".xray-pre")?.textContent || stdout || "";
    body.innerHTML = `${badge}<pre class="xray-pre">${escapeHtml(existing)}</pre>`;
  }
  const sub = el.querySelector(".stage-sub");
  if (sub) sub.textContent = `stdout · exit ${exitCode}`;
  el.scrollIntoView({ behavior: "smooth", block: "end" });
}

// ── GUI status polling ────────────────────────────────────────────────────────

let _guiRunning = false;
let _guiBusy    = false;

function _applyGuiState({ running, ready, busy }) {
  _guiRunning = running;
  _guiBusy    = busy;

  els.guiDot.className = "status-dot" + (running && ready ? " on" : running ? " starting" : "");
  els.guiLabel.textContent = running
    ? (busy ? "GUI BUSY" : ready ? "GUI READY" : "GUI STARTING…")
    : "GUI OFFLINE";

  els.startGui.hidden = running;
  els.killGui.hidden  = !running;

  els.runSim.disabled = !(running && ready && !busy);
  els.runSim.title    = running
    ? (busy ? "Robot is busy" : !ready ? "GUI still starting" : "Send command to PyBullet")
    : "Start the GUI first";

  els.guiHint.textContent = running
    ? (busy ? "Robot is executing — wait for READY."
            : ready ? "GUI is live. Type a command and click SEND TO SIM."
                    : "GUI window is opening…")
    : "Start the PyBullet window first, then type commands below.";
}

async function pollGuiState() {
  try {
    const r = await fetch("/api/sim/state");
    if (r.ok) _applyGuiState(await r.json());
  } catch (_) {}
}

setInterval(pollGuiState, 1500);
pollGuiState();

// ── START / KILL GUI ──────────────────────────────────────────────────────────

els.startGui.addEventListener("click", async () => {
  els.startGui.disabled = true;
  els.startGui.textContent = "STARTING…";
  try {
    const r = await fetch("/api/sim/start", { method: "POST" });
    const j = await r.json();
    if (!r.ok) throw new Error(j.detail || "start failed");
    logInfo("PyBullet GUI launching — window will open shortly…");
    pollGuiState();
  } catch (e) {
    logError(String(e.message || e));
  } finally {
    els.startGui.disabled = false;
    els.startGui.textContent = "▶ START GUI";
  }
});

els.killGui.addEventListener("click", async () => {
  if (!confirm("Kill the PyBullet GUI window?")) return;
  try {
    await fetch("/api/sim/stop", { method: "POST" });
    logInfo("GUI process terminated.");
    pollGuiState();
  } catch (e) { logError(String(e.message || e)); }
});

// ── Shared pipeline runner (used by both buttons) ─────────────────────────────

async function runPipelineStages(task) {
  // INTAKE — appears immediately
  xrayAppend({ id: "intake", label: "OPERATOR // INTAKE",
    subtitle: "Raw natural-language command", detail: task, accent: "blue" });

  // GEMINI — call, then append result when ready
  let geminiPlan = "";
  try {
    const gr = await fetch("/api/stage/gemini", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ task }),
    });
    const gj = await gr.json();
    if (!gr.ok) throw new Error(gj.detail || "Gemini failed");
    geminiPlan = gj.plan;
    xrayAppend({ id: "gemini", label: "INPUT INTERPRETATION // GEMINI",
      subtitle: "High-level numbered action sequence (no code)",
      detail: geminiPlan, accent: "blue" });
  } catch (err) {
    xrayAppend({ id: "gemini", label: "INPUT INTERPRETATION // GEMINI",
      subtitle: "FAILED", detail: String(err.message || err), accent: "red" });
    throw err;
  }

  // K2 — call, then append result when ready
  let k2Code = "", parsedActions = [];
  try {
    const kr = await fetch("/api/stage/k2", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ plan: geminiPlan }),
    });
    const kj = await kr.json();
    if (!kr.ok) throw new Error(kj.detail || "K2 failed");
    k2Code = kj.code;
    parsedActions = kj.parsed || [];
    xrayAppend({ id: "k2", label: "ORCHESTRATION ENGINE // K2",
      subtitle: "Executable primitive calls (Python surface)",
      detail: k2Code, accent: "red" });
  } catch (err) {
    xrayAppend({ id: "k2", label: "ORCHESTRATION ENGINE // K2",
      subtitle: "FAILED", detail: String(err.message || err), accent: "red" });
    throw err;
  }

  // QUEUE — always append after K2
  xrayAppend({ id: "queue", label: "ACTUATION QUEUE // PARSED",
    subtitle: "Lines accepted for eval() in sim / robot bridge",
    lines: parsedActions, accent: "red" });

  return { geminiPlan, k2Code, parsedActions };
}

// ── RUN PIPELINE ──────────────────────────────────────────────────────────────

function setLoading(on) {
  els.submitBtn.disabled = on;
  els.submitBtn.textContent = on ? "TRANSMITTING…" : "RUN PIPELINE";
}

els.form.addEventListener("submit", async (e) => {
  e.preventDefault();
  const task = els.task.value.trim();
  if (!task) return;

  els.task.value = ""; // auto-clear
  logCmd(task);
  logWorking("Calling Gemini planner…");
  setLoading(true);
  xrayClear();

  try {
    const { parsedActions, k2Code } = await runPipelineStages(task);
    const n = parsedActions.length;
    logReady(`Plan compiled · ${n} action${n !== 1 ? "s" : ""} queued`);
    els.lastJson = {
      trace: { task, compiled_code: k2Code, parsed_actions: parsedActions },
      exported_at: new Date().toISOString(),
    };
  } catch (err) {
    logError(String(err.message || err).slice(0, 140));
  } finally {
    setLoading(false);
  }
});

// ── SEND TO SIM ───────────────────────────────────────────────────────────────

els.runSim.addEventListener("click", async () => {
  const task = els.task.value.trim();
  if (!task) { alert("Enter a command first."); return; }
  if (!_guiRunning) { logError("GUI not running — click START GUI first."); return; }
  if (_guiBusy)     { logError("Robot is busy — wait for READY."); return; }

  els.task.value = ""; // auto-clear
  logCmd(task);
  logWorking("Building plan…");
  xrayClear();
  pollGuiState();

  let parsedActions = [], k2Code = "";
  try {
    const res = await runPipelineStages(task);
    parsedActions = res.parsedActions;
    k2Code        = res.k2Code;
  } catch (err) {
    logError(`Planning failed: ${String(err.message || err).slice(0, 100)}`);
    return;
  }

  // Dispatch to sim
  logWorking("Sending to PyBullet sim…");
  let prevLineCount = 0;

  try {
    const r = await fetch("/api/sim/run", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ task }),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.detail || "dispatch failed");

    const jobId = j.job_id;
    logInfo(`Job ${jobId} dispatched`);
    pollGuiState();

    // Stream stdout line-by-line into the sim stage
    const logPoller = setInterval(async () => {
      try {
        const lr = await fetch("/api/sim/log");
        const lj = await lr.json();
        const lines = lj.lines || [];
        // Only append NEW lines since last poll
        for (let i = prevLineCount; i < lines.length; i++) {
          xrayAppendSimLine(lines[i]);
        }
        prevLineCount = lines.length;
      } catch (_) {}
    }, 500);

    // Poll for job completion
    const jobPoller = setInterval(async () => {
      pollGuiState();
      try {
        const pr = await fetch(`/api/sim/poll/${jobId}`);
        const pj = await pr.json();
        if (!pj.done) return;

        clearInterval(jobPoller);
        clearInterval(logPoller);

        // Flush any remaining lines
        const lr = await fetch("/api/sim/log");
        const lj = await lr.json();
        const lines = lj.lines || [];
        for (let i = prevLineCount; i < lines.length; i++) {
          xrayAppendSimLine(lines[i]);
        }

        const exitCode = pj.returncode ?? 0;
        xrayFinalSim(exitCode, "");

        if (exitCode === 0) logReady("Sim complete · exit 0 · Ready for next command");
        else                logError(`Sim exited ${exitCode} — see X-ray`);

        els.lastJson = {
          trace: { task, compiled_code: k2Code, parsed_actions: parsedActions },
          exported_at: new Date().toISOString(),
        };
        pollGuiState();
      } catch (_) {}
    }, 1500);

  } catch (err) {
    logError(String(err.message || err).slice(0, 140));
    pollGuiState();
  }
});

// ── Clear log ─────────────────────────────────────────────────────────────────

els.clearLog.addEventListener("click", () => {
  els.sessionLog.innerHTML = '<div class="session-empty">Awaiting commands.</div>';
});

// ── Adapter ───────────────────────────────────────────────────────────────────

async function loadAdapter() {
  try {
    const r = await fetch("/api/robot/adapter");
    const j = await r.json();
    if (els.adapterUrl)     els.adapterUrl.value = j.endpoint || "";
    if (els.adapterEnabled) els.adapterEnabled.checked = !!j.enabled;
  } catch (_) {}
}

els.saveAdapter.addEventListener("click", async () => {
  let headers = {};
  const raw = (els.adapterHeaders?.value || "").trim();
  if (raw) { try { headers = JSON.parse(raw); } catch { alert("Headers must be valid JSON."); return; } }
  await fetch("/api/robot/adapter", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      endpoint: els.adapterUrl?.value.trim() || "",
      headers, enabled: !!els.adapterEnabled?.checked,
    }),
  });
  els.saveAdapter.textContent = "SAVED";
  setTimeout(() => { els.saveAdapter.textContent = "COMMIT ADAPTER"; }, 900);
});

// ── Copy trace ────────────────────────────────────────────────────────────────

els.copyTrace.addEventListener("click", async () => {
  if (!els.lastJson) { alert("Run the pipeline first."); return; }
  try {
    await navigator.clipboard.writeText(JSON.stringify(els.lastJson, null, 2));
    els.copyTrace.textContent = "COPIED";
    setTimeout(() => { els.copyTrace.textContent = "COPY FULL TRACE (JSON)"; }, 900);
  } catch { alert("Clipboard unavailable."); }
});

loadAdapter();
