# Carbon — Language-Driven Robotic Manipulation

A full-stack system that lets you control a robotic arm using plain English. Type a command in the web UI, watch the reasoning pipeline unfold in real time, and see the robot execute it — all live.

---

## System Architecture

The pipeline has four phases, each building on the last.

---

### Phase 1 — Semantic 3D Modeling · *"What is this?"*

The robotic arm is physically blind. When you say *"grab the coffee mug by the handle,"* the robot sees only a raw 3D point cloud — a chaotic cloud of depth measurements from a depth sensor.

**This implementation uses a Microsoft Kinect + Franka Panda robotic arm.** In a production deployment this stage would use a LiDAR sensor for higher-resolution, longer-range scanning — the rest of the pipeline is sensor-agnostic.

1. **3D Scan** — The depth sensor builds a point cloud of the workspace.
2. **Semantic Injection** — A vision-language model (CLIP / Segment Anything 3D) "paints" meaning onto the 3D map, identifying clusters of points that correspond to real-world concepts: *"This cluster is a Mug."*
3. **Geometric Decomposition** — The identified cluster is broken down into primitive mathematical shapes. A mug becomes a hollow cylinder (the cup body) attached to a half-torus (the handle). This gives the system a structured, math-friendly description of every object in the scene.

Relevant files: `clip_model.py`, `segmentation_3d.py`, `reconstruction.py`, `scene_representation.py`, `scene.py`

---

### Phase 2 — Affordance Mapping · *"How do I interact with this?"*

Knowing *what* an object is is not enough — the robot also needs to know *how* to interact with it. Affordance mapping answers this.

An **affordance** is a physical property of an object that dictates what actions it supports:

- *Cylinders with flat tops* → afford pushing or wrapping
- *Torus shapes* → afford hooking or pinching

The system applies these rules to the decomposed geometry from Phase 1. The result is an **affordance map** — a spatial annotation of the scene that tells the orchestrator exactly where to reach, push, or grasp for any given object.

Relevant files: `affordance_map.py`, `pipeline.py`

---

### Phase 3 — LLM Orchestration · *"What should I do?"*

The LLM does not calculate motor angles. It acts as the **logic engine** — it reads your natural-language command alongside the affordance map and produces a sequence of high-level skill calls.

The pipeline is two-stage:

| Stage | Model | Role |
|---|---|---|
| **Input Interpretation** | Gemini | Translates your command into a numbered, safety-ordered natural-language plan (hover → grasp → lift → move → release) |
| **Orchestration Engine** | K2 | Compiles that plan into executable Python primitive calls (`grasp()`, `lift()`, `push()`, `carry()`, `flip()`, `drop()`) |

**Example:**

```
Prompt:  "Grab the mug by the handle."

Gemini → 1. Hover above mug.
          2. Drop down to mug.
          3. Grasp mug.
          4. Lift mug.

K2     → grasp()
          lift(0.4)
```

The compiled primitives are passed directly to the execution layer.

Relevant files: `llm.py`, `main.py`, `web/serve.py`

---

### Phase 4 — Execution · *"How do I move?"*

This is where the software brain translates into physical motion.

**Inverse Kinematics (IK) — macro movement**
IK is rigid, deterministic math. Given a target coordinate (from the affordance map), IK calculates exactly how to position every joint in the arm to place the gripper at the right spot without colliding with the table or other objects.

**Skill Primitives — micro movement**
Once the gripper is in position, the primitive library takes over. Each primitive (`grasp`, `push`, `flip`, etc.) handles the fine-grained control — contact detection, grip force, orientation changes, constraint binding — built on top of PyBullet's physics simulation.

**Available primitives:**

| Primitive | Description |
|---|---|
| `grasp()` | Safe 3-phase approach (clearance → hover → descend) + grip with contact detection |
| `lift(height)` | Raise arm vertically by `height` metres |
| `carry(direction, distance)` | Move a held object horizontally through the air |
| `push(direction, distance)` | Slide a free object along the table surface |
| `flip()` | Rotate held object 180° with automatic clearance check |
| `drop()` | Open gripper and release object at current position |

Relevant files: `actions.py`, `robot.py`

---

## Web Control Surface

A real-time browser UI that runs the full pipeline and surfaces every reasoning step.

```
http://127.0.0.1:8765
```

**X-Ray Panel** — Every layer of reasoning is visible as it happens:
1. `OPERATOR // INTAKE` — your raw command
2. `INPUT INTERPRETATION // GEMINI` — Gemini's numbered plan (appears when Gemini responds)
3. `ORCHESTRATION ENGINE // K2` — compiled primitive calls (appears when K2 responds)
4. `ACTUATION QUEUE // PARSED` — final list of actions sent to the robot
5. `SUBPROCESS // PYBULLET` — live stdout from the simulation as the robot moves

**Session Log** — Timestamped history of every command, status update, and exit code.

**GUI Lifecycle** — `START GUI` opens the PyBullet simulation window. The window stays open indefinitely so you can watch the robot. `KILL GUI` closes it. `SEND TO SIM` pipes your command to the running window without reopening it.

---

## Setup

### Requirements

```
pip install pybullet
pip install fastapi "uvicorn[standard]" requests
pip install google-generativeai
```

Or install everything at once:

```
pip install -r requirements-frontend.txt
```

### API Keys Required

You need two API keys. Add them to `llm.py` before running:

| Key | Where to get it | Variable in `llm.py` |
|---|---|---|
| **Gemini API Key** | [Google AI Studio](https://aistudio.google.com/app/apikey) | `GEMINI_API_KEY` |
| **K2 API Key** | [K2Think](https://k2think.ai) | `K2_API_KEY` |

> **Important:** Never commit `llm.py` with real keys. It is excluded from version control for this reason.

---

## Running

**Terminal mode** (command-line interface):

```bash
python main.py
```

Type commands at the `>` prompt. The robot executes them in the PyBullet GUI.

**Web UI mode** (recommended for demos):

```bash
python -m uvicorn web.serve:app --host 127.0.0.1 --port 8765
```

Then open `http://127.0.0.1:8765` in your browser. Click **START GUI** to open the simulation window, then type commands and click **SEND TO SIM**.

---

## Repository Structure

```
├── actions.py              Robot skill primitives (grasp, push, carry, flip, drop…)
├── llm.py                  Gemini + K2 pipeline (add your API keys here)
├── main.py                 Terminal-mode command loop
├── sim_persistent.py       Long-running PyBullet process for the web UI
├── sim_once.py             Single-shot sim runner (legacy / testing)
│
├── affordance_map.py       Affordance rules and object-action mapping
├── clip_model.py           CLIP vision-language model integration
├── pipeline.py             End-to-end perception pipeline
├── reconstruction.py       3D point cloud reconstruction
├── scene.py                Scene management
├── scene_representation.py Structured scene object model
├── segmentation_3d.py      3D semantic segmentation
├── robot.py                Low-level robot interface
├── utils.py                Shared utilities
│
├── web/
│   ├── serve.py            FastAPI server (plan, stage, sim endpoints)
│   └── static/
│       ├── index.html      Control surface UI
│       ├── app.js          Live X-ray and session log logic
│       └── styles.css      Cyberpunk terminal styling
│
├── requirements-frontend.txt
└── README.md
```
