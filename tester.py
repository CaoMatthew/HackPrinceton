import os
import time

import pybullet as p
import pybullet_data
import requests
from dotenv import load_dotenv

import actions
from scene import scene

load_dotenv()

K2_API_KEY = os.getenv("K2_API_KEY")
K2_API_URL = os.getenv("K2_API_URL")


def create_cylinder(position, radius=0.04, height=0.2, mass=1):
    collision_shape = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=radius,
        height=height,
    )
    visual_shape = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=radius,
        length=height,
        rgbaColor=[0.2, 0.4, 0.9, 1.0],
    )
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
    )


def setup_world():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(0)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=50,
        cameraPitch=-35,
        cameraTargetPosition=[0.5, 0, 0],
    )

    scene["mug"]["id"] = p.loadURDF("cube_small.urdf", [0.5, 0.2, 0.1])
    scene["box"]["id"] = p.loadURDF("cube_small.urdf", [0.7, 0, 0.1])
    scene["tube_large"]["id"] = create_cylinder([0.2, 0.4, 0.1], radius=0.05, height=0.2)
    scene["target_zone"]["id"] = p.loadURDF(
        "cube_small.urdf",
        [0.8, 0.3, 0.05],
        globalScaling=1.5,
    )

    return robot


def draw_point(pos, color):
    offset = 0.03
    p.addUserDebugLine(
        [pos[0] - offset, pos[1], pos[2]],
        [pos[0] + offset, pos[1], pos[2]],
        color,
        2,
    )
    p.addUserDebugLine(
        [pos[0], pos[1] - offset, pos[2]],
        [pos[0], pos[1] + offset, pos[2]],
        color,
        2,
    )
    p.addUserDebugLine(
        [pos[0], pos[1], pos[2] - offset],
        [pos[0], pos[1], pos[2] + offset],
        color,
        2,
    )


def draw_scene_points():
    colors = {
        "handle": [1, 0, 0],
        "top": [0, 1, 0],
        "body": [0, 0, 1],
        "center": [1, 1, 0],
    }

    for obj_data in scene.values():
        for part, pos in obj_data.items():
            if part == "id":
                continue
            draw_point(pos, colors.get(part, [1, 1, 1]))


def generate_plan(task):
    system_prompt = """
You are a robot planner.

Convert the user task directly into Python function calls.

Available functions:
- move_to(target)
- grasp(target)
- lift(height)
- place(target)

Rules:
- Only use these functions
- Use object targets like "mug.handle" or "target_zone.center"
- Output ONLY valid Python code
- One function call per line
- No explanations
"""

    response = requests.post(
        K2_API_URL,
        headers={
            "Authorization": f"Bearer {K2_API_KEY}",
            "Content-Type": "application/json",
            "accept": "application/json",
        },
        json={
            "model": "MBZUAI-IFM/K2-Think-v2",
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": task},
            ],
            "stream": False,
        },
    )

    if response.status_code != 200:
        raise RuntimeError(f"K2 API Error: {response.text}")

    result = response.json()
    output = result["choices"][0]["message"]["content"].strip()
    return output.replace("```python", "").replace("```", "").strip()


def execute_plan(code):
    env = {
        "move_to": actions.move_to,
        "grasp": actions.grasp,
        "lift": actions.lift,
        "place": actions.place,
    }

    for raw_line in code.splitlines():
        line = raw_line.strip()
        if not line:
            continue

        print("Executing:", line)
        try:
            eval(line, {}, env)
            time.sleep(0.5)
        except Exception as exc:
            print("Execution error:", exc)


def main():
    # 1. Setup simulation + robot
    robot = setup_world()

    # 2. Draw debug points (so you can SEE targets)
    draw_scene_points()

    # 3. Initialize actions with robot ID (CRITICAL)
    actions.init(robot)
    #TEMP
    actions.move_to("mug.handle")

    print("Tester mode: enter a task and K2 will turn it into action calls.")
    print("Example: pick up the mug by the handle and place it in the target zone")

    while True:
        task = input("\nTask (or 'q' to quit): ").strip()

        if task.lower() == "q":
            break

        try:
            # 4. Generate plan from LLM
            plan = generate_plan(task)
            print("\nK2 plan:\n", plan)

            # 5. Execute the plan (this calls move_to, etc.)
            execute_plan(plan)

        except Exception as exc:
            print("Planner error:", exc)

    # 6. Clean shutdown
    p.disconnect()
    print("Tester ended.")

    print("Tester mode: enter a task and K2 will turn it into action calls.")
    print("Gemini is not used in this file.")
    print("Example: pick up the mug by the handle and place it in the target zone")

    while True:
        task = input("\nTask (or 'q' to quit): ").strip()
        if task.lower() == "q":
            break

        try:
            plan = generate_plan(task)
            print("\nK2 plan:\n", plan)
            execute_plan(plan)
        except Exception as exc:
            print("Planner error:", exc)

    p.disconnect()
    print("Tester ended.")


if __name__ == "__main__":
    main()
