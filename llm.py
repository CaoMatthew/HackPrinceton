import os
import requests
import google.generativeai as genai
from dotenv import load_dotenv
import warnings
warnings.filterwarnings("ignore")

# --- Load environment variables ---
load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
K2_API_KEY = os.getenv("K2_API_KEY")
K2_API_URL = os.getenv("K2_API_URL")

# --- Configure Gemini ---
genai.configure(api_key=GEMINI_API_KEY)
gemini_model = genai.GenerativeModel("gemini-2.5-flash")


# =========================================================
# ⚡ STEP 1: Gemini → Structured Natural Language Plan
# =========================================================
def gemini_plan(task: str) -> str:
    prompt = f"""
You are a robot task planner.

Break the task into clear, numbered steps.

Rules:
- Be concise
- Use object names like "mug handle"
- Be explicit about parts (e.g., handle, top)
- Do NOT write code
- Do NOT explain anything

Task:
{task}
"""

    response = gemini_model.generate_content(prompt)
    plan_text = response.text.strip()

    return plan_text


# =========================================================
# 🧠 STEP 2: K2 → Compile Plan into Function Calls
# =========================================================
def k2_compile(plan_text: str) -> str:
    system_prompt = """
You are a robot planner.

Convert the plan into Python function calls.

Available functions:
- move_to(target)
- grasp(target)
- lift(height)
- place(target)

Rules:
- Only use these functions
- Use "mug.handle" format
- Output ONLY valid Python code
- One function per line
- No explanations
"""

    print("K2 URL:", K2_API_URL)

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
            "stream": False  # ✅ IMPORTANT
        }
    )

    if response.status_code != 200:
        raise Exception(f"K2 API Error: {response.text}")

    result = response.json()

    # extract response safely
    output = result["choices"][0]["message"]["content"].strip()

    # clean formatting
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