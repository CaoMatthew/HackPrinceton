"""
HTTP frontend for HackPrinceton: planning trace (X-ray) + optional robot hooks.
Does not modify llm.py / main.py — imports planning functions from llm.

Run from repo root:
  python -m uvicorn web.serve:app --host 127.0.0.1 --port 8765
"""
from __future__ import annotations

import subprocess
import sys
import threading
import uuid
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import requests
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from llm import gemini_plan, k2_compile

STATIC = Path(__file__).resolve().parent / "static"

app = FastAPI(title="HackPrinceton Control Surface")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── Pluggable robotics backends ───────────────────────────────────────────────
_robot_adapter: dict[str, Any] = {"endpoint": "", "headers": {}, "enabled": False}

# ── Persistent GUI process state ─────────────────────────────────────────────
_gui: dict[str, Any] = {
    "proc":    None,   # subprocess.Popen or None
    "ready":   False,  # True once "READY" line received from stdout
    "busy":    False,  # True while a command is executing
    "log":     [],     # captured stdout lines
}

# ── One-shot job registry (for polling) ──────────────────────────────────────
_jobs: dict[str, dict[str, Any]] = {}


# ── Models ────────────────────────────────────────────────────────────────────

class PlanRequest(BaseModel):
    task: str = Field(..., min_length=1)
    forward_to_robot: bool = False


class RobotAdapterBody(BaseModel):
    endpoint: str = ""
    headers: dict[str, str] = {}
    enabled: bool = False


class SimRequest(BaseModel):
    task: str = Field(..., min_length=1)


VALID_CALLS = ("grasp(", "lift(", "carry(", "drop(", "place(", "flip(", "push(")


def _parse_action_lines(code: str) -> list[str]:
    return [
        line.split("#")[0].strip()
        for line in code.split("\n")
        if any(line.split("#")[0].strip().startswith(c) for c in VALID_CALLS)
    ]


# ── Persistent GUI helpers ────────────────────────────────────────────────────

def _gui_running() -> bool:
    p = _gui["proc"]
    return p is not None and p.poll() is None


def _read_gui_stdout(proc: subprocess.Popen) -> None:
    """Background thread: drain stdout from the persistent sim process."""
    for raw in proc.stdout:
        line = raw.rstrip()
        _gui["log"].append(line)
        if line == "READY":
            _gui["ready"] = True
        elif line == "COMMAND_DONE":
            _gui["busy"] = False


# ── Health ────────────────────────────────────────────────────────────────────

@app.get("/api/health")
def health():
    return {"ok": True, "service": "hackprinceton-web"}


# ── Robot adapter ─────────────────────────────────────────────────────────────

@app.get("/api/robot/adapter")
def get_adapter():
    return {
        "endpoint": _robot_adapter["endpoint"],
        "enabled":  _robot_adapter["enabled"],
        "headers_set": bool(_robot_adapter.get("headers")),
    }


@app.post("/api/robot/adapter")
def set_adapter(body: RobotAdapterBody):
    _robot_adapter["endpoint"] = (body.endpoint or "").strip()
    _robot_adapter["headers"]  = dict(body.headers or {})
    _robot_adapter["enabled"]  = bool(body.enabled and _robot_adapter["endpoint"])
    return get_adapter()


# ── Individual stage endpoints (for live X-ray streaming) ────────────────────

class GeminiRequest(BaseModel):
    task: str = Field(..., min_length=1)


class K2Request(BaseModel):
    plan: str = Field(..., min_length=1)


@app.post("/api/stage/gemini")
def stage_gemini(req: GeminiRequest):
    """Run only the Gemini planner and return the plan text."""
    try:
        text = gemini_plan(req.task.strip())
        return {"plan": text}
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e)) from e


@app.post("/api/stage/k2")
def stage_k2(req: K2Request):
    """Run only the K2 compiler on a given plan and return code + parsed actions."""
    try:
        code = k2_compile(req.plan)
        parsed = _parse_action_lines(code)
        return {"code": code, "parsed": parsed}
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e)) from e


@app.get("/api/sim/log")
def sim_log():
    """Return current stdout lines from the running persistent sim (partial or full)."""
    return {"lines": list(_gui.get("log", [])), "busy": _gui.get("busy", False)}


# ── Plan (Gemini → K2 only, no sim) ──────────────────────────────────────────

@app.post("/api/plan")
def plan(req: PlanRequest):
    task = req.task.strip()
    if not task:
        raise HTTPException(status_code=400, detail="empty task")

    stages: list[dict[str, Any]] = [{
        "id": "intake", "label": "OPERATOR // INTAKE",
        "subtitle": "Raw natural-language command (pre-schema)",
        "detail": task, "accent": "blue",
    }]

    try:
        gemini_text = gemini_plan(task)
    except Exception as e:
        raise HTTPException(status_code=502, detail=f"Gemini planner failed: {e}") from e

    stages.append({
        "id": "gemini", "label": "INPUT INTERPRETATION // GEMINI",
        "subtitle": "High-level numbered action sequence (no code)",
        "detail": gemini_text, "accent": "blue",
    })

    try:
        k2_code = k2_compile(gemini_text)
    except Exception as e:
        raise HTTPException(status_code=502, detail=f"K2 compiler failed: {e}") from e

    stages.append({
        "id": "k2", "label": "ORCHESTRATION ENGINE // K2",
        "subtitle": "Executable primitive calls (Python surface)",
        "detail": k2_code, "accent": "red",
    })

    parsed = _parse_action_lines(k2_code)
    stages.append({
        "id": "queue", "label": "ACTUATION QUEUE // PARSED",
        "subtitle": "Lines accepted for eval() in the simulator / robot bridge",
        "lines": parsed,
        "detail": "\n".join(parsed) if parsed else "(no matching primitives)",
        "accent": "red",
    })

    payload = {"task": task, "stages": stages,
               "compiled_code": k2_code, "parsed_actions": parsed}

    if req.forward_to_robot and _robot_adapter["enabled"]:
        url = _robot_adapter["endpoint"]
        try:
            r = requests.post(url, headers=_robot_adapter.get("headers") or {},
                              json={"task": task, "gemini_plan": gemini_text,
                                    "compiled_code": k2_code, "parsed_actions": parsed},
                              timeout=15)
            stages.append({"id": "robot_tx", "label": "ROBOT ADAPTER // TX",
                           "subtitle": f"POST {url}",
                           "detail": f"HTTP {r.status_code}\n{r.text[:8000]}",
                           "accent": "blue"})
            payload["robot_dispatch"] = {"status_code": r.status_code, "body": r.text[:4000]}
        except Exception as e:
            stages.append({"id": "robot_tx", "label": "ROBOT ADAPTER // TX",
                           "subtitle": "Forward failed", "detail": str(e), "accent": "red"})
            payload["robot_dispatch"] = {"error": str(e)}

    return payload


# ── Persistent GUI management ─────────────────────────────────────────────────

@app.post("/api/sim/start")
def start_gui():
    """Spawn the persistent PyBullet GUI process."""
    if _gui_running():
        return {"status": "already_running"}

    script = ROOT / "sim_persistent.py"
    if not script.is_file():
        raise HTTPException(status_code=500, detail="sim_persistent.py missing")

    try:
        proc = subprocess.Popen(
            [sys.executable, str(script)],
            cwd=str(ROOT),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # merge stderr into stdout
            text=True,
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

    _gui["proc"]  = proc
    _gui["ready"] = False
    _gui["busy"]  = False
    _gui["log"]   = []

    threading.Thread(target=_read_gui_stdout, args=(proc,), daemon=True).start()
    return {"status": "started"}


@app.post("/api/sim/stop")
def stop_gui():
    """Send __QUIT__ to the persistent sim then terminate it."""
    proc = _gui.get("proc")
    if proc and proc.poll() is None:
        try:
            proc.stdin.write("__QUIT__\n")
            proc.stdin.flush()
        except Exception:
            pass
        proc.terminate()
    _gui["proc"]  = None
    _gui["ready"] = False
    _gui["busy"]  = False
    return {"status": "stopped"}


@app.get("/api/sim/state")
def gui_state():
    """Current state of the persistent GUI process."""
    running = _gui_running()
    return {
        "running": running,
        "ready":   running and _gui.get("ready", False),
        "busy":    running and _gui.get("busy", False),
    }


# ── Send a command to the persistent GUI ─────────────────────────────────────

@app.post("/api/sim/run")
def run_sim(req: SimRequest):
    """Pipe a command to the running GUI; returns job_id for polling."""
    if not _gui_running():
        raise HTTPException(status_code=400,
                            detail="GUI not started. Click START GUI first.")
    if not _gui.get("ready"):
        raise HTTPException(status_code=503,
                            detail="GUI is still starting up. Wait a moment.")
    if _gui.get("busy"):
        raise HTTPException(status_code=429,
                            detail="Robot is busy. Wait for the current command to finish.")

    task = req.task.strip()
    job_id = str(uuid.uuid4())[:8]

    _gui["busy"] = True
    _gui["log"]  = []   # clear log for this command

    try:
        _gui["proc"].stdin.write(task + "\n")
        _gui["proc"].stdin.flush()
    except Exception as e:
        _gui["busy"] = False
        raise HTTPException(status_code=500, detail=f"Failed to send command: {e}")

    _jobs[job_id] = {"done": False, "returncode": None, "stdout": "", "task": task}

    def _wait_done(jid: str) -> None:
        import time as _t
        while _gui.get("busy") and _gui_running():
            _t.sleep(0.3)
        _jobs[jid]["done"]       = True
        _jobs[jid]["returncode"] = 0 if _gui_running() else 1
        _jobs[jid]["stdout"]     = "\n".join(_gui.get("log", []))

    threading.Thread(target=_wait_done, args=(job_id,), daemon=True).start()
    return {"job_id": job_id, "status": "dispatched"}


@app.get("/api/sim/poll/{job_id}")
def poll_sim(job_id: str):
    """Poll a dispatched sim job."""
    job = _jobs.get(job_id)
    if not job:
        raise HTTPException(status_code=404, detail="unknown job")
    if not job["done"]:
        return {"job_id": job_id, "done": False}
    return {"job_id": job_id, "done": True,
            "returncode": job["returncode"], "stdout": job["stdout"], "stderr": ""}


# ── Static files ──────────────────────────────────────────────────────────────

@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")


def main():
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8765)


if __name__ == "__main__":
    main()
