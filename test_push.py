"""
Test: Push Forward
==================
Tests the push skill: arm navigates beside the block and pushes it forward (+X).
Logs per-step contact forces — broken down by robot link — to push_contact.log
so arm-table and arm-block collisions can be diagnosed precisely.

Run: python test_push.py
"""
import pybullet as p
import pybullet_data
import time
import csv
from collections import defaultdict
import actions

# Franka link index -> human-readable name (base is -1, arm 0-6, hand 7-8, fingers 9-10)
LINK_NAMES = {
    -1: "link0_base",
     0: "link1",
     1: "link2",
     2: "link3",
     3: "link4_elbow",
     4: "link5",
     5: "link6",
     6: "link7_wrist",
     7: "link8",
     8: "hand",
     9: "leftfinger",
    10: "rightfinger",
    11: "grasptarget",
}

# --- Setup ---
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# --- Scene ---
table = p.loadURDF("table/table.urdf", [0.75, 0, 0], globalScaling=0.5)
block = p.loadURDF("cube_small.urdf", [0.75, 0, 0.35])

# --- Gravity + settle ---
p.setGravity(0, 0, -9.8)
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

actual_pos, _ = p.getBasePositionAndOrientation(block)
print(f"Block settled at: {[f'{x:.4f}' for x in actual_pos]}")

# --- Camera ---
p.resetDebugVisualizerCamera(
    cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
    cameraTargetPosition=[0.75, 0, 0.3]
)

# --- Contact logger ---
_step_counter = [0]
log_rows = []

def _contacts_summary(contacts):
    """Return (count, {link_idx: max_normal_force}, peak_force) from contact list."""
    by_link = defaultdict(float)
    for c in contacts:
        link = c[3]   # link index on bodyA (robot)
        by_link[link] = max(by_link[link], abs(c[9]))
    peak = max(by_link.values()) if by_link else 0.0
    return len(contacts), dict(by_link), peak

def contact_step_callback(step_idx):
    ee_pos, _ = actions.get_ee_pos()

    robot_table = p.getContactPoints(bodyA=robot, bodyB=table) or []
    robot_block = p.getContactPoints(bodyA=robot, bodyB=block) or []

    t_count, t_by_link, t_peak = _contacts_summary(robot_table)
    b_count, b_by_link, b_peak = _contacts_summary(robot_block)

    # Compact string: "{link_idx/name: force, ...}"
    def fmt_links(by_link):
        return "{" + ", ".join(
            f"{LINK_NAMES.get(k, k)}: {v:.1f}N"
            for k, v in sorted(by_link.items())
        ) + "}" if by_link else "{}"

    log_rows.append({
        "global_step":       _step_counter[0],
        "phase":             actions._motion_phase,
        "phase_step":        step_idx,
        "ee_x":              f"{ee_pos[0]:.4f}",
        "ee_y":              f"{ee_pos[1]:.4f}",
        "ee_z":              f"{ee_pos[2]:.4f}",
        "table_contacts":    t_count,
        "table_peak_N":      f"{t_peak:.2f}",
        "table_by_link":     fmt_links(t_by_link),
        "block_contacts":    b_count,
        "block_peak_N":      f"{b_peak:.2f}",
        "block_by_link":     fmt_links(b_by_link),
    })
    _step_counter[0] += 1

actions._step_callback = contact_step_callback

# --- Run test ---
actions.init(robot, block)
actions.push(direction="forward", distance=0.15)

# --- Write CSV log ---
LOG_FILE = "push_contact.log"
fieldnames = [
    "global_step", "phase", "phase_step",
    "ee_x", "ee_y", "ee_z",
    "table_contacts", "table_peak_N", "table_by_link",
    "block_contacts", "block_peak_N", "block_by_link",
]
with open(LOG_FILE, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(log_rows)

print(f"\nContact log written -> {LOG_FILE}  ({len(log_rows)} steps)")

# --- Summarise by phase + link ---
print("\n=== TABLE COLLISION SUMMARY (by phase + link) ===")
# Aggregate: phase -> link_name -> [forces]
phase_link_forces = defaultdict(lambda: defaultdict(list))
first_hit = {}   # phase -> first step_idx where contact occurs

for r in log_rows:
    if int(r["table_contacts"]) == 0:
        continue
    phase = r["phase"]
    if phase not in first_hit:
        first_hit[phase] = (r["phase_step"], r["ee_x"], r["ee_y"], r["ee_z"])
    raw = r["table_by_link"].strip("{}")
    for part in raw.split(","):
        part = part.strip()
        if ":" not in part:
            continue
        name, force_str = part.split(":")
        phase_link_forces[phase][name.strip()].append(
            float(force_str.strip().rstrip("N"))
        )

if not phase_link_forces:
    print("  [OK] No robot-table contacts during push.\n")
else:
    for phase, links in sorted(phase_link_forces.items()):
        fstep, fx, fy, fz = first_hit.get(phase, ("?", "?", "?", "?"))
        print(f"\n  phase={phase}")
        print(f"    first contact: phase_step={fstep}  EE=({fx}, {fy}, {fz})")
        for link, forces in sorted(links.items()):
            print(f"    {link:<20s}  steps={len(forces):4d}  "
                  f"peak={max(forces):8.1f} N  avg={sum(forces)/len(forces):7.1f} N")

print("\n=== BLOCK CONTACT SUMMARY (by phase + link) ===")
bphase_link_forces = defaultdict(lambda: defaultdict(list))
for r in log_rows:
    if int(r["block_contacts"]) == 0:
        continue
    phase = r["phase"]
    raw = r["block_by_link"].strip("{}")
    for part in raw.split(","):
        part = part.strip()
        if ":" not in part:
            continue
        name, force_str = part.split(":")
        bphase_link_forces[phase][name.strip()].append(
            float(force_str.strip().rstrip("N"))
        )

if not bphase_link_forces:
    print("  No robot-block contacts logged.\n")
else:
    for phase, links in sorted(bphase_link_forces.items()):
        print(f"\n  phase={phase}")
        for link, forces in sorted(links.items()):
            print(f"    {link:<20s}  steps={len(forces):4d}  "
                  f"peak={max(forces):8.1f} N  avg={sum(forces)/len(forces):7.1f} N")

# --- Result ---
final_pos, _ = p.getBasePositionAndOrientation(block)
print(f"\nBlock started at:  {[f'{x:.4f}' for x in actual_pos]}")
print(f"Block ended at:    {[f'{x:.4f}' for x in final_pos]}")
print(f"Displacement X:    {final_pos[0] - actual_pos[0]:.4f}m  (target: +0.15m)")

print("\n[OK] Push test complete")

p.setRealTimeSimulation(0)
p.disconnect()
