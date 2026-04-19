import os
import re
import requests
import google.generativeai as genai
import warnings
warnings.filterwarnings("ignore")

GEMINI_API_KEY = "AIzaSyACqKTs6AV4NzK5x3dHL8Gn9B6SM6ooBEE"
K2_API_KEY = os.getenv("K2_API_KEY", "")
K2_API_URL = os.getenv("K2_API_URL", "")

# --- Configure Gemini ---
genai.configure(api_key=GEMINI_API_KEY)
gemini_model = genai.GenerativeModel("gemini-2.5-flash")


# =========================================================
# STEP 1: Gemini → Structured Natural Language Plan
# =========================================================
def gemini_plan(task: str) -> str:
    prompt = f"""
You are a robot task planner for a Franka Panda arm in a simulation.

Scene: one cube/block sitting on a table in front of the robot.

Available robot actions:
- grasp()          — close gripper on the block
- lift(height)     — lift the block to a height in metres (e.g. 0.4)
- place()          — release the block at the current position
- flip()           — flip the block upside-down
- push(direction, distance)  — slide the block along the table
    direction: "forward" (+X away from robot), "left" (-Y), "right" (+Y)
    distance: metres to slide (e.g. 0.15)

Rules:
- Output ONLY a numbered list of steps, nothing else.
- Each step maps to exactly one action above.
- "back" is not a valid push direction — use pick-and-place instead.
- Be explicit: e.g. "Push the block forward 0.15 m" or "Grasp the block".
- Do NOT write code. Do NOT explain. Do NOT add headers.

Task: {task}
"""
    response = gemini_model.generate_content(prompt)
    return response.text.strip()


# =========================================================
# STEP 2: K2 → Compile Plan into Function Calls
# =========================================================
def k2_compile(plan_text: str) -> str:
    system_prompt = """
You are a robot code compiler. Convert a natural-language numbered plan into
Python function calls, one per line.

Available functions (exact signatures):
  grasp()
  lift(height)          # height in metres, e.g. lift(0.4)
  place()
  flip()
  push(direction, distance)
    # direction must be one of: "forward", "left", "right"
    # distance in metres, e.g. push("forward", 0.15)

Rules:
- Output ONLY bare Python function calls, one per line.
- No imports, no variables, no comments, no explanations.
- Do not invent function names not listed above.
- If the plan says "back", translate it to a lift() + place() sequence.
- Preserve all numeric values exactly as stated in the plan.
"""
    response = requests.post(
        K2_API_URL,
        headers={
            "Authorization": f"Bearer {K2_API_KEY}",
            "Content-Type": "application/json",
            "accept": "application/json"
        },
        json={
            "model": "MBZUAI-IFM/K2-Think-v2",
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": plan_text}
            ],
            "stream": False
        }
    )

    if response.status_code != 200:
        raise Exception(f"K2 API Error: {response.text}")

    output = response.json()["choices"][0]["message"]["content"].strip()
    output = re.sub(r'<think>.*?</think>', '', output, flags=re.DOTALL).strip()
    output = output.replace("```python", "").replace("```", "").strip()
    return output


# =========================================================
# 🔗 MAIN PIPELINE FUNCTION
# =========================================================
def generate_plan(task: str) -> str:
    print("\n--- USER TASK ---")
    print(task)

    # Step 1: Gemini
    gemini_output = gemini_plan(task)
    print("\n--- GEMINI PLAN ---")
    print(gemini_output)

    # Step 2: K2
    k2_output = k2_compile(gemini_output)
    print("\n--- K2 COMPILED CODE ---")
    print(k2_output)

    return k2_output