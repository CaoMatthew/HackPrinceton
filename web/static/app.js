const $ = (sel, root = document) => root.querySelector(sel);

const els = {
  form: $("#plan-form"),
  task: $("#task"),
  forward: $("#forward-robot"),
  runSim: $("#run-sim"),
  submitBtn: $("#submit-btn"),
  xray: $("#xray-timeline"),
  empty: $("#xray-empty"),
  sessionLog: $("#session-log"),
  adapterUrl: $("#adapter-url"),
  adapterHeaders: $("#adapter-headers"),
  adapterEnabled: $("#adapter-enabled"),
  saveAdapter: $("#save-adapter"),
  copyTrace: $("#copy-trace"),
  lastJson: null,
};

const MAX_SESSION = 12;

function pushSessionLine(task) {
  if (!els.sessionLog) return;
  const empty = els.sessionLog.querySelector(".session-empty");
  if (empty) empty.remove();
  const ts = new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" });
  const row = document.createElement("div");
  row.className = "session-entry";
  row.innerHTML = `<span class="ts">${escapeHtml(ts)}</span><span class="msg">${escapeHtml(task)}</span>`;
  els.sessionLog.insertBefore(row, els.sessionLog.firstChild);
  while (els.sessionLog.children.length > MAX_SESSION) {
    els.sessionLog.removeChild(els.sessionLog.lastChild);
  }
}

function setLoading(loading) {
  els.submitBtn.disabled = loading;
  els.runSim.disabled = loading;
  els.submitBtn.textContent = loading ? "TRANSMITTING…" : "RUN PIPELINE";
}

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;");
}

function renderStages(stages) {
  els.empty.style.display = "none";
  els.xray.removeAttribute("hidden");
  els.xray.innerHTML = "";
  for (const st of stages) {
    const accent = st.accent || "blue";
    const tag = accent === "red" ? "RED CHANNEL" : "BLUE CHANNEL";
    const detail = st.lines
      ? `<ul class="action-list">${st.lines.map((l) => `<li>${escapeHtml(l)}</li>`).join("")}</ul>`
      : `<pre class="xray-pre">${escapeHtml(st.detail || "")}</pre>`;
    const sub = st.subtitle
      ? `<p class="stage-sub">${escapeHtml(st.subtitle)}</p>`
      : "";
    const block = document.createElement("article");
    block.className = "stage";
    block.dataset.accent = accent;
    block.innerHTML = `
      <div class="stage-head">
        <div>
          <h3 class="stage-title">${escapeHtml(st.label)}</h3>
          ${sub}
        </div>
        <span class="tag accent">${tag}</span>
      </div>
      <div class="stage-body">${detail}</div>
    `;
    els.xray.appendChild(block);
  }
}

async function loadAdapter() {
  try {
    const r = await fetch("/api/robot/adapter");
    const j = await r.json();
    els.adapterUrl.value = j.endpoint || "";
    els.adapterEnabled.checked = !!j.enabled;
  } catch (_) {
    /* ignore */
  }
}

async function saveAdapter() {
  let headers = {};
  const raw = els.adapterHeaders.value.trim();
  if (raw) {
    try {
      headers = JSON.parse(raw);
    } catch (e) {
      alert("Headers must be valid JSON object.");
      return;
    }
  }
  const body = {
    endpoint: els.adapterUrl.value.trim(),
    headers,
    enabled: els.adapterEnabled.checked,
  };
  await fetch("/api/robot/adapter", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
}

els.form.addEventListener("submit", async (e) => {
  e.preventDefault();
  const task = els.task.value.trim();
  if (!task) return;
  setLoading(true);
  els.xray.innerHTML = "";
  els.xray.setAttribute("hidden", "");
  els.empty.style.display = "block";
  els.empty.textContent = "Establishing uplink to planners…";
  try {
    const r = await fetch("/api/plan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ task, forward_to_robot: els.forward.checked }),
    });
    const text = await r.text();
    let j;
    try {
      j = JSON.parse(text);
    } catch {
      throw new Error(text || "Bad JSON from server");
    }
    if (!r.ok) throw new Error(j.detail || text);
    pushSessionLine(task);
    els.lastJson = { trace: j, exported_at: new Date().toISOString() };
    renderStages(j.stages || []);
  } catch (err) {
    els.empty.style.display = "block";
    els.empty.innerHTML = `<div class="err">${escapeHtml(String(err.message || err))}</div>`;
  } finally {
    setLoading(false);
  }
});

els.runSim.addEventListener("click", async () => {
  const task = els.task.value.trim();
  if (!task) {
    alert("Enter a command first.");
    return;
  }
  if (
    !confirm(
      "This launches the local PyBullet window (sim_once.py) and may take a minute. Continue?"
    )
  ) {
    return;
  }
  setLoading(true);
  try {
    const r = await fetch("/api/sim/run", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ task }),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.detail || "sim failed");
    const extra = {
      id: "sim",
      label: "SUBPROCESS // PYBULLET",
      subtitle: "stdout/stderr from sim_once.py (full mechanical trace)",
      detail: (j.stdout || "") + (j.stderr ? `\n--- STDERR ---\n${j.stderr}` : ""),
      accent: "blue",
    };
    const prev = els.lastJson?.trace;
    const base = (prev?.stages || []).filter((s) => s.id !== "sim");
    const mergedStages = [...base, extra];
    els.lastJson = {
      ...(els.lastJson || {}),
      trace: { ...(prev || {}), stages: mergedStages, sim_subprocess: j },
      exported_at: new Date().toISOString(),
    };
    renderStages(mergedStages);
  } catch (err) {
    alert(String(err.message || err));
  } finally {
    setLoading(false);
  }
});

els.saveAdapter.addEventListener("click", async () => {
  try {
    await saveAdapter();
    els.saveAdapter.textContent = "SAVED";
    setTimeout(() => {
      els.saveAdapter.textContent = "COMMIT ADAPTER";
    }, 900);
  } catch (e) {
    alert(String(e));
  }
});

els.copyTrace.addEventListener("click", async () => {
  if (!els.lastJson) {
    alert("Run the pipeline first.");
    return;
  }
  try {
    await navigator.clipboard.writeText(JSON.stringify(els.lastJson, null, 2));
    els.copyTrace.textContent = "COPIED";
    setTimeout(() => {
      els.copyTrace.textContent = "COPY FULL TRACE (JSON)";
    }, 900);
  } catch {
    alert("Clipboard unavailable.");
  }
});

loadAdapter();
