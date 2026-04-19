"""
HTTP frontend for HackPrinceton: planning trace (X-ray) + optional robot hooks.
Does not modify llm.py / main.py — imports planning functions from llm.

Run from repo root:
  python -m uvicorn web.serve:app --host 127.0.0.1 --port 8765
Or:
  cd web && python serve.py
"""
from __future__ import annotations

import subprocess
import sys
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

# --- Pluggable robotics backends (in-memory; restart clears) ---
_robot_adapter: dict[str, Any] = {
    "endpoint": "",
    "headers": {},
    "enabled": False,
}


class PlanRequest(BaseModel):
    task: str = Field(..., min_length=1)
    forward_to_robot: bool = False


class RobotAdapterBody(BaseModel):
    endpoint: str = ""
    headers: dict[str, str] = {}
    enabled: bool = False


VALID_CALLS = ("grasp(", "lift(", "place(", "flip(", "push(")


def _parse_action_lines(code: str) -> list[str]:
    lines_out: list[str] = []
    for line in code.split("\n"):
        line = line.split("#")[0].strip()
        if any(line.startswith(c) for c in VALID_CALLS):
            lines_out.append(line)
    return lines_out


@app.get("/api/health")
def health():
    return {"ok": True, "service": "hackprinceton-web"}


@app.get("/api/robot/adapter")
def get_adapter():
    return {
        "endpoint": _robot_adapter["endpoint"],
        "enabled": _robot_adapter["enabled"],
        "headers_set": bool(_robot_adapter.get("headers")),
    }


@app.post("/api/robot/adapter")
def set_adapter(body: RobotAdapterBody):
    _robot_adapter["endpoint"] = (body.endpoint or "").strip()
    _robot_adapter["headers"] = dict(body.headers or {})
    _robot_adapter["enabled"] = bool(body.enabled and _robot_adapter["endpoint"])
    return get_adapter()


@app.post("/api/plan")
def plan(req: PlanRequest):
    task = req.task.strip()
    if not task:
        raise HTTPException(status_code=400, detail="empty task")

    stages: list[dict[str, Any]] = [
        {
            "id": "intake",
            "label": "OPERATOR // INTAKE",
            "subtitle": "Raw natural-language command (pre-schema)",
            "detail": task,
            "accent": "blue",
        }
    ]

    try:
        gemini_text = gemini_plan(task)
    except Exception as e:
        raise HTTPException(status_code=502, detail=f"Gemini planner failed: {e}") from e

    stages.append(
        {
            "id": "gemini",
            "label": "SEMANTIC PLANNER // GEMINI",
            "subtitle": "High-level numbered action sequence (no code)",
            "detail": gemini_text,
            "accent": "blue",
        }
    )

    try:
        k2_code = k2_compile(gemini_text)
    except Exception as e:
        raise HTTPException(status_code=502, detail=f"K2 compiler failed: {e}") from e

    stages.append(
        {
            "id": "k2",
            "label": "SKILL COMPILER // K2",
            "subtitle": "Executable primitive calls (Python surface)",
            "detail": k2_code,
            "accent": "red",
        }
    )

    parsed = _parse_action_lines(k2_code)
    stages.append(
        {
            "id": "queue",
            "label": "ACTUATION QUEUE // PARSED",
            "subtitle": "Lines accepted for eval() in the simulator / robot bridge",
            "lines": parsed,
            "detail": "\n".join(parsed) if parsed else "(no matching primitives)",
            "accent": "red",
        }
    )

    payload = {
        "task": task,
        "stages": stages,
        "compiled_code": k2_code,
        "parsed_actions": parsed,
    }

    if req.forward_to_robot and _robot_adapter["enabled"]:
        url = _robot_adapter["endpoint"]
        try:
            r = requests.post(
                url,
                headers=_robot_adapter.get("headers") or {},
                json={
                    "task": task,
                    "gemini_plan": gemini_text,
                    "compiled_code": k2_code,
                    "parsed_actions": parsed,
                },
                timeout=15,
            )
            stages.append(
                {
                    "id": "robot_tx",
                    "label": "ROBOT ADAPTER // TX",
                    "subtitle": f"POST {url}",
                    "detail": f"HTTP {r.status_code}\n{r.text[:8000]}",
                    "accent": "blue",
                }
            )
            payload["robot_dispatch"] = {"status_code": r.status_code, "body": r.text[:4000]}
        except Exception as e:
            stages.append(
                {
                    "id": "robot_tx",
                    "label": "ROBOT ADAPTER // TX",
                    "subtitle": "Forward failed",
                    "detail": str(e),
                    "accent": "red",
                }
            )
            payload["robot_dispatch"] = {"error": str(e)}

    return payload


class SimRequest(BaseModel):
    task: str = Field(..., min_length=1)


@app.post("/api/sim/run")
def run_sim(req: SimRequest):
    """Spawn one-shot PyBullet run (opens GUI). Stdout is returned for X-ray."""
    task = req.task.strip()
    exe = sys.executable
    script = ROOT / "sim_once.py"
    if not script.is_file():
        raise HTTPException(status_code=500, detail="sim_once.py missing")

    try:
        proc = subprocess.run(
            [exe, str(script), task],
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            timeout=600,
        )
    except subprocess.TimeoutExpired as e:
        raise HTTPException(status_code=504, detail=f"simulation timeout: {e}") from e

    return {
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
    }


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")


def main():
    import uvicorn

    uvicorn.run(app, host="127.0.0.1", port=8765)


if __name__ == "__main__":
    main()
