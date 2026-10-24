# DUM-E — 4-DOF Robot Arm Kinematics Core

A single-file mathematics + visualization layer for a 4-DOF robot arm
(`Base → fixed L0 → joint1/L1 → joint2/L2 → joint3/wrist mount → L3 →
gripper mount → Fingers`). The model is a fixed vertical `L0 = 100 mm` base
link, a `400 mm` L1 upper link, a `300 mm` L2 forearm, and an `80 mm` L3 wrist
stem. **The four DOF are `theta_base`, `joint1`, `joint2` and `joint3`** — `joint3`
is the wrist and is a real, commandable joint (`±150°`, default `-60°`), *not* a
fixed kink. The `WRIST_STEM_ANGLE = -60°` config field is a legacy value used by
`_grip_parameters` for reporting; live FK/IK use the `joint3` you pass in. L0 cannot
tilt; the base rotates about world `+Z`; joint1 and joint2 control radial-Z tilt
and have no independent spin. At the gripper mount sits a simulated **3-finger
gripper** (`GRIPPER_FINGERS = 3`): three rigid **curled talons** — each a
`FINGER_LENGTH`/`Lf` (150 mm) circular arc of fixed curvature — spaced 120° around the
tool axis, whose on-axis **grip point** (where a gripped object sits) is the
end-effector that FK/IK/reachability use. The opening (`GRIPPER_OPENING`, 0 =
closed with the tips meeting on the axis, 1 = fully open) only **swings** each
rigid finger about its base: it never bends the finger and never moves the grip
point.

> **Reach envelope:** the envelope depends on `joint3`, because the effective
> forearm `m` is measured from `L2` along a direction set by that joint. It does
> **not** depend on the gripper opening — the grasp depth is constant, so there is
> one envelope per `joint3`. Reachable radial-Z distances are measured from the
> joint1 pivot at `(0, 0, L0)`, not from the world origin. With the default
> `joint3 = -60°` the range is `39.15 … 839.15 mm`; the **maximum** over all
> `joint3` is `104.05 … 904.05 mm` at `joint3 = 0°`. See
> [§8](#8-workspace-and-joint-limits--two-separate-ideas) for the full table.

**Stage scope:** forward kinematics, inverse kinematics, workspace validation,
joint-limit validation, FK→IK→FK verification, a 3D visualization — plus
**smooth joint-space animations** between poses (CLI option `7` and the GUI
`Animate` button), in which the gripper opening is animated *in step* with the
arm rather than snapping — available both as a text CLI (`robot_arm.py`) and as a
**tkinter desktop GUI** (`robot_arm_gui.py`) with the 5 workflow stages.
Deliberately **not** implemented: Arduino, serial, servo control, camera,
vision, speech, LLM/AI, web, databases, motion planning.

---

## Contents

1. Requirements
2. Quick start
3. Using it as a library
4. How to use it — concrete walkthrough (CLI, script, GUI)
5. Coordinate system
6. Forward Kinematics (FK)
7. Inverse Kinematics (IK)
8. Workspace and joint limits — two separate ideas
9. FK → IK → FK verification
10. Visualization
11. Implementation notes
12. Everything explained — concepts, math and full API reference

---

## 1. Requirements

- Python 3.10+
- `numpy`
- `matplotlib`
- `tkinter` (for the GUI tab; ships with the standard Windows/macOS
  installers, on Debian/Ubuntu: `sudo apt install python3-tk`)

```bash
pip install numpy matplotlib
```

The kinematics interface uses these files:
- `robot_arm.py` — the kinematics core (import it as a library, run it as a CLI).
- `robot_arm_gui.py` — the tkinter desktop UI (imports `robot_arm`).
- `../robot_contract.py` — shared joint limits for the robot interfaces.

---

## 2. Quick start

```bash
python robot_arm.py
```

Draw the arm at `base yaw=30°, joint1/L1=45°, joint2/L2=-40°`:

```
================================
       DUM-E 4-DOF ROBOT
================================

1. Forward Kinematics
2. Inverse Kinematics
3. Check Reachability
4. Visualize From Joint Angles
5. Visualize From XYZ Target
6. Run FK -> IK Verification
7. Animate To Target (smooth)
8. Exit
================================
```

| Option | Purpose |
| --- | --- |
| `1` | Joint angles (deg) → grip point `(x, y, z)` mm + the 3 fingertip positions |
| `2` | Target `(x, y, z)` mm → joint angles (deg), with joint2-pose choice and limit check |
| `3` | Is a target reachable? (the reach depends on `joint3`, not the opening) |
| `4` | Joint angles → 3D plot of the robot + fingers |
| `5` | Target `(x, y, z)` → IK → 3D plot of the robot + target |
| `6` | Run the FK→IK→FK verification suite |
| `7` | Smooth transition from the last drawn pose to an IK target, easing the joints and the gripper opening together |
| `8` | Exit |

Options `1`–`7` additionally ask for an optional **gripper opening** (0..1;
Enter keeps the config default `gripper_opening`); option `7` also asks for
the **animation frame count** (Enter keeps 30) and eases the opening from the
last drawn opening to the target opening.

---

## 3. Using it as a library

```python
import robot_arm as ra

# FK: degrees in, millimetres out (grip point = object centre).
# joint3 is a REQUIRED positional argument.
p = ra.forward_kinematics(30.0, 45.0, -40.0, -60.0)      # CartesianPoint(x, y, z)

# Optional gripper opening, 0 (closed) .. 1 (open); defaults to gripper_opening
p = ra.forward_kinematics(30.0, 45.0, -40.0, -60.0, opening=0.0)

# Full geometry: joint positions, gripper_mount, fingertips, grip point
gp = ra.gripper_pose(30.0, 45.0, -40.0, -60.0)           # FingerGripper
tips = ra.finger_tips(30.0, 45.0, -40.0, -60.0)          # tuple of 3 CartesianPoint

# IK: millimetres in, degrees out; the target is a world-frame grip point.
# Pass joint3="auto" to let the solver pick it, or "down" to pin the stem vertical.
angles = ra.inverse_kinematics(300.0, 120.0, 150.0, joint2_up=True)  # JointAngles
down = ra.inverse_kinematics(300.0, 120.0, 150.0, joint3="down")    # JointAngles
down.joint1 + down.joint2 + down.joint3                            # -> -90.0

# Workspace + limits (is_reachable also takes joint3, default -60 deg)
ra.is_reachable(300.0, 120.0, 150.0)          # bool
ra.check_joint_limits(30.0, 45.0, -40.0, -60.0)   # bool

# Interactive 3D visualization (JointAngles takes four fields)
ra.plot_robot(ra.JointAngles(30.0, 45.0, -40.0, -60.0),
              target=ra.CartesianPoint(300.0, 120.0, 150.0))

# Smooth joint-space transition between two poses
ra.interpolate_joints(ra.JointAngles(30.0, 45.0, -40.0, -60.0),
                      ra.JointAngles(45.0, 30.0, -20.0, -60.0), frames=60)
ra.plot_animation(ra.JointAngles(30.0, 45.0, -40.0, -60.0),
                  ra.JointAngles(45.0, 30.0, -20.0, -60.0),
                  target=ra.CartesianPoint(150.0, 60.0, 80.0), frames=60)

# Verification (optional opening argument)
ra.verify_fk_ik(n_samples=50)
```

### Class-based usage

`RobotArm` binds a `RobotConfig` and exposes the same four-joint operations without
requiring `config` on every call. `joint3` is required for FK, gripper geometry, and
joint-limit checks; reachability and IK default to `joint3=-60°`.

```python
import robot_arm as ra

arm = ra.RobotArm()
grip = arm.forward_kinematics(30.0, 45.0, -40.0, -60.0)
pose = arm.gripper_pose(30.0, 45.0, -40.0, -60.0, opening=0.5)
angles = arm.inverse_kinematics(grip.x, grip.y, grip.z, joint3="auto")
reachable = arm.is_reachable(grip.x, grip.y, grip.z, joint3=-60.0)
within_limits = arm.check_joint_limits(
    angles.theta_base, angles.joint1, angles.joint2, angles.joint3
)
```

The facade's `is_reachable` method accepts `joint3` in degrees, like the other
class-based joint-angle methods. For `joint3="auto"`, `joint2_up` is preferred first;
if that branch has no limit-valid solution for any wrist candidate, the solver tries the
alternate elbow branch.

---

## 4. How to use it — concrete walkthrough

### 4.1 CLI, end to end

From the project folder:

```bash
python robot_arm.py
```

The menu appears. **Option `1` — Forward Kinematics** asks for four joint
angles in degrees (then an optional gripper opening) and returns the **grip
point** plus every fingertip:

```
Select option: 1
  theta_base     (deg): 45
  joint1         (deg): 30
  joint2         (deg): -20
  joint3         (deg): -60
  gripper opening 0..1 (default 1, Enter = default):

  Grip point: x=546.603 mm, y=546.603 mm, z=195.784 mm  (opening 1.000)
  Finger 1 tip: x=605.968 mm, y=504.682 mm, z=181.083 mm
  Finger 2 tip: x=496.406 mm, y=547.049 mm, z=141.215 mm
  Finger 3 tip: x=563.601 mm, y=614.243 mm, z=220.952 mm
```

(The grip point is the same for **any** opening — opening only swings the
rigid fingers. The pose `45, 30, -20, -60` with opening `0` closes the tips on
the axis at the same grip point `x=546.603, y=546.603, z=195.784` mm.)


**Option `2` — Inverse Kinematics** asks for a target in millimetres (plus an
optional opening) and returns the angles (it also reports whether the solution
respects the joint limits). The target is the **grip point**, i.e. the object
centre the fingers enclose:

```
Select option: 2
  x (mm): 300
  y (mm): 120
  z (mm): 150
  joint2 pose [up/down] (up): up
  gripper opening 0..1 (default 1, Enter = default):

  theta_base=  21.801 deg
  joint1=  82.351 deg
  joint2= -110.705 deg
  joint3= -60.000 deg
  within joint limits: yes
```

Try the *down* configuration for the same target — you get a different pose
(`joint1= 81.569 deg, joint2= -109.740 deg, joint3= -61.830 deg`) whose grip point is
still exactly `(300, 120, 150)`. Try option `3` with `(1200, 0, 0)`: it is
**UNREACHABLE** at the default `joint3 = -60°`, because its 1204.159 mm distance from
the joint1 pivot exceeds `L1 + m = 839.15 mm`. It is unreachable at every `joint3`:
the largest outer bound is `904.05 mm`, at `joint3 = 0°`. The inner bound at the
default `joint3` is 39.15 mm, so `(100, 0, 0)` is reachable geometrically from the
`z = L0` pivot; check the joint limits before relying on it.


**Option `5` — Visualize From XYZ Target** opens a matplotlib window
(3 fingers drawn in cyan, plus a thin dotted red connector from the wrist to
the target). Confirm the drawn pose reaches the target you asked for: run
option `5` with the same `(300, 120, 150)` and check the recon-FK line:

```
  IK result: base=21.801, joint1=82.351, joint2=-110.705 (deg)
  FK(recomputed): x=300.000, y=120.000, z=150.000 mm
```

**Option `6` — FK → IK verification** runs the automated suite and prints a
PASS/FAIL summary (optionally for a specific opening):

```
========================================================
FK -> IK -> FK verification: PASS
Position error: 0.000000 mm  (tolerance 0.1 mm)
Max angle round-trip error: 0.000000 deg
Samples: 40/40 passed
========================================================
```

**Option `7` — Animate To Target** solves IK for a target and then plays a
joint-space transition from the current pose to the IK pose, with the elbow
branch fixed for the whole move so the arm cannot flip mid-animation and the
grip opening eased in step with the joints. Targets outside the geometric
workspace are rejected with an error.

**Option `8`** quits.

### 4.2 As a library / script

Create a file `demo.py` alongside `robot_arm.py`:

```python
"""Minimal DUM-E demo: FK, IK, workspace, limits, then a 3D plot."""
import robot_arm as ra

def main() -> None:
    # 1) Forward kinematics: angles (deg) -> grip point (mm)
    pose = ra.JointAngles(45.0, 30.0, -20.0, -60.0)
    grip = ra.forward_kinematics(pose.theta_base, pose.joint1, pose.joint2, pose.joint3)
    print(f"FK(45, 30, -20, -60) -> x={grip.x:.3f} y={grip.y:.3f} z={grip.z:.3f}")

    # Finger geometry for the same pose (opening defaults to gripper_opening=1)
    gp = ra.gripper_pose(pose.theta_base, pose.joint1, pose.joint2, pose.joint3)
    print(f"   spread={gp.spread:.1f} deg, {len(gp.fingertips)} fingertips")
    print(f"   wrist = ({gp.joint3.x:.3f}, {gp.joint3.y:.3f}, {gp.joint3.z:.3f}) mm")

    # 2) Inverse kinematics: grip point (mm) -> angles (deg), joint2 pose up/down
    target = ra.CartesianPoint(300.0, 120.0, 150.0)
    up = ra.inverse_kinematics(target.x, target.y, target.z, joint2_up=True)
    down = ra.inverse_kinematics(target.x, target.y, target.z, joint2_up=False)
    print(f"IK up   -> base={up.theta_base:7.3f}  joint1={up.joint1:7.3f}  joint2={up.joint2:7.3f}")
    print(f"IK down -> base={down.theta_base:7.3f}  joint1={down.joint1:7.3f}  joint2={down.joint2:7.3f}")

    # 3) Sanity: FK(IK(target)) reproduces the target for both modes
    for sol in (up, down):
        back = ra.forward_kinematics(sol.theta_base, sol.joint1, sol.joint2, sol.joint3)
        err = ((back.x - target.x) ** 2 + (back.y - target.y) ** 2 + (back.z - target.z) ** 2) ** 0.5
        print(f"   round-trip error: {err:.2e} mm")

    # 4) Workspace and limits (joint3 changes the reach; the opening does not)
    print("reachable(300,120,150):", ra.is_reachable(target.x, target.y, target.z))
    print("reachable(700,0,0):    ", ra.is_reachable(700.0, 0.0, 0.0))
    print("within limits (up):     ", ra.check_joint_limits(
        up.theta_base, up.joint1, up.joint2, up.joint3))

    # 5) Interactive 3D plot with the target marked and fingers drawn
    ra.plot_robot(up, target=target)


if __name__ == "__main__":
    main()
```

Run it:

```bash
python demo.py
```

Expected console output (a matplotlib window then opens):

```
FK(45, 30, -20, -60) -> x=546.603 y=546.603 z=195.784
   spread=30.0 deg, 3 fingertips
   wrist = (490.220, 490.220, 290.811) mm
IK up   -> base= 21.801  joint1= 82.351  joint2=-110.705
IK down -> base= 21.801  joint1=-64.758  joint2= 158.161
   round-trip error: 8.53e-14 mm
   round-trip error: 6.51e-14 mm
reachable(300,120,150): True
reachable(700,0,0):     True
within limits (up):     True
```

> `plot()`/`plot_robot()` call `plt.show()` and block until the window is
> closed — that is expected behaviour, not a hang.

### 4.3 Desktop GUI (5 workflow stages)

```bash
python robot_arm_gui.py
```

Opens a native tkinter window with a notebook of the **5 core stages** — type
values into the fields, press the button, read the result:

| Tab | What you do | What you get |
| --- | --- | --- |
| **FK** | joint angles (deg) + opening | grip point, wrist, spread, 3 fingertips |
| **IK** | grip-point target (mm) + opening + joint2 pose | joint angles, limit check, round-trip error |
| **Reachability** | target (mm) + joint3 | reachable?, `d`, reach `[min, max]` bounds for the chosen `joint3` (opening shown but inert) |
| **Verification** | `n_samples` + opening | FK→IK→FK PASS/FAIL suite output |
| **Visualization** | joint angles **or** XYZ target + opening | embedded 3D plot of the arm, fingers (cyan) and a thin dotted red connector to the target; an `Animate` button eases the joints and the grip opening from the current pose |

Notes:

- Every field labelled `opening 0..1` may be left blank — blank keeps the
  config default `gripper_opening` (1.0).
- The **FK** and **IK** tabs have a `Visualize` button that fills the
  Visualization tab and draws the pose/target for you.
- The Visualization tab embeds the same renderer as `plot_robot()` (both call
  the shared `robot_arm.draw_robot()`), so what you see shows exactly what the
  CLI plots — with the matplotlib toolbar for rotating/zooming in 3D.
- The **`Animate`** button plays an eased, joint-space transition from the
  last drawn pose (the next run continues from where the previous one ended);
  the gripper opening eases from its current value to the target alongside the
  joints. It becomes `Stop` while running, so you can cancel mid-motion.
- On Linux you may need `python3-tk` installed (see the requirements
  section); otherwise the GUI reports a friendly error and the CLI remains
  fully usable.
- Invalid numbers and out-of-range openings are caught and shown as `ERROR`
  in the tab (or a message box).

### 4.4 Animate To Target — smooth transitions

**Option `7`** solves IK for a target (as in options `2`/`5`) and then plays
a *smooth animation* in joint space from the **last pose you drew** to that
target pose. Interpolation uses an ease-in/out curve (accelerate, cruise,
decelerate) and, for `theta_base`, the shortest angular path — so
`170° → -170°` sweeps through `180°`, not through `0°`. The gripper opening
rides the same curve, so the fingers open/close *during* the move instead of
snapping to the target opening on the first frame.

```
Select option: 7
  x (mm): 300
  y (mm): 120
  z (mm): 150
  joint2 pose [up/down] (up): up
  gripper opening 0..1 (default 1, Enter = default): 1
  animation frames (default 30, Enter = default):

  Animating transition:
    start  base=  45.000  joint1=  30.000  joint2= -20.000  joint3= -60.000 deg  opening=1.000
    target base=  21.801  joint1=  82.351  joint2=-110.705  joint3= -60.000 deg  opening=1.000
```

A matplotlib window opens and the arm eases from the start pose to the
target (whose thin dotted red connector stays visible, so you see the hand land on it).
When it closes, the target pose becomes the new "last drawn" pose — run
option `7` again to chain another transition from where the arm just ended.
If the target pose and opening are *both* identical to the last drawn pose
(e.g. you re-enter the same target you just drew), option `7` says so instead
of opening an empty animation.

---

## 5. Coordinate system

Right-handed Cartesian frame:

```
             Z (up)
             ↑
             |
             |
             ●────────→ X
            /
           /
          Y
```

- The base is the only world-frame yaw joint; it rotates about **+Z** and
  cannot tilt the fixed vertical L0 link.
- L0 runs from the base origin `(0, 0, 0)` to the joint1 pivot `(0, 0, L0)`.
- After the base turns, joint1/L1 and joint2/L2 move in the *radial–Z plane*,
  the vertical plane through the current arm azimuth. L3 keeps its fixed
  orientation relative to L2 and has no independent spin.

### Angle conventions (all internally in **radians**, public API in **degrees**)

| Angle | Meaning | Zero at | Positive direction |
| --- | --- | --- | --- |
| `theta_base` | rotation of the whole arm about global `+Z` | arm along `+X` | counter-clockwise viewed from above |
| `joint1` | link-1 angle in the radial–Z plane | link-1 horizontal, pointing outward | link-1 rises (`+Z`) |
| `joint2` | link-2 angle **relative to** link-1 | straight (fully extended) | forearm folds past link-1 (see note) |

> **Signed joint-2 sign note:** with `joint1` positive when the upper
> arm rises, the *joint2-up* IK solution uses a **negative** `joint2`
> (forearm folds back and down over the raised upper arm); the *joint2-down*
> solution uses a positive `joint2`. The `joint2_up` flag in
> `inverse_kinematics` selects between them.

**Signed radial projection:** `r` in the FK equations is a *signed*
projection, not a distance. When `|joint1 + joint2| > 90°`, the
gripper swings past the vertical axis and ends up "behind" the base
(`r < 0`). Its true azimuth is then `theta_base + 180°`. IK always returns a
front-facing pose with `theta_base = atan2(y, x)`, so it still reaches the
point, but it may not reproduce the original angles of a backward-folded pose.

**Wrist joint (`joint3`):** the `L3` stem points `joint3` degrees off the forearm
direction (default `-60°`), so there is a visible kink at the wrist-mount marker.
`joint3` is the fourth DOF, so the stem points *wherever you command it* — but
within a single pose it is rigid, so `joint2` moves the stem *together with* the
forearm. That is what makes the planar solver closed-form. The stem runs from
the **wrist mount** (end of L2) to the **gripper mount** (end of L3).
Mathematically the tip behaves like a plain 2-link arm whose forearm has magnitude
and phase:

```
m     = |L2 + L3·e^(i·joint3)|        (≈ 346.99 mm at joint3 −60°)
delta = arg(L2 + L3·e^(i·joint3))     (≈ −11.52° at joint3 −60°)
```

**Gripper fingers & grip point:** `GRIPPER_FINGERS = 3` rigid **curled talons**
sit on the **gripper mount**, azimuthally spaced 120° around the tool axis. Each
is one solid piece — a circular arc of fixed curvature, `FINGER_LENGTH`/`Lf` (150 mm)
of arc length, curling inward by `FINGER_CURL_DEG` (60°). The opening only
**swings** the whole rigid finger about its base; it never bends it:

```
spread   = GRIPPER_OPENING · FINGER_OPENING_DEG       (default 1 → 30° swing)
R_arc    = FINGER_LENGTH / radians(FINGER_CURL_DEG)   = 143.24 mm
R_base   = R_arc · (1 − cos(FINGER_CURL_DEG))         = 71.62 mm  (base ring)
l_g      = R_arc · sin(FINGER_CURL_DEG)    # grip depth  = 124.05 mm  (constant)
```

The three bases sit on a ring of radius `R_base` rather than one point, so at
`opening = 0` every tip meets the axis at depth `l_g` and the three tips and the
grip point coincide. `R_base` and the tip radius are the values that make a
curled finger close *onto* the axis instead of through it.

The **grip point** `G = gripper_mount + l_g·axis` is the on-axis grasp centre —
where a gripped object sits and the target that FK/IK/reachability use. Because
`G` sits on the tool axis, it folds into the same planar picture: the on-axis
extension `(L3 + l_g)` replaces `L3` above, giving the effective forearm

```
a_g     = L3 + l_g                       # stem plus grip depth, rigid together
m_g     = |L2 + a_g·e^(i·phi)|            (≈ 439.15 mm @ joint3 −60°)
delta_g = arg(L2 + a_g·e^(i·phi))         (≈ −23.73° @ joint3 −60°)
```

Two things follow, and both matter:

**`phi` is `joint3`, not the `WRIST_STEM_ANGLE` constant.** The stem and the grip
depth rotate together with the wrist, so `m_g` and `delta_g` are functions of the
*commanded* `joint3`. That is what makes the reach envelope in
[§8](#8-workspace-and-joint-limits--two-separate-ideas) joint3-dependent. The table
below is evaluated at the default `joint3 = −60°`; every `m_g` figure elsewhere in
this document assumes the same.

**The opening does not enter here.** `l_g` is the constant `GRIP_DEPTH = 124.05 mm`,
so `m_g` and `delta_g` do *not* depend on the opening. Earlier revisions derived
`l_g` from the spread (`l_g = FINGER_LENGTH·cos(spread)`), which made the grip
depth — and therefore the reach and the solved joints — change as the gripper
opened. See
[issue 15](../README.md#15-opening-once-leaked-into-the-kinematic-chain) in the
root README.

---

## 6. Forward Kinematics (FK)

```text
forward_kinematics(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG, opening=None)
# -> CartesianPoint(x, y, z) [mm]
```

Returns the **grip point** (on-axis grasp centre). Note `joint3` is a
required positional argument. With `l_g = GRIP_DEPTH` (a constant) and `phi` = the
`joint3` argument (in radians):

```
r = L1·cos(joint1) + L2·cos(joint1 + joint2) + (L3 + l_g)·cos(joint1 + joint2 + joint3)
x = r·cos(theta_base),        y = r·sin(theta_base)
z = L0 + L1·sin(joint1) + L2·sin(joint1 + joint2) + (L3 + l_g)·sin(joint1 + joint2 + joint3)
```

`joint1`, `joint2` and `joint3` are in degrees here and converted internally.
Link 1 points at `joint1`, link 2 at `joint1 + joint2`, and the on-axis
extension (stem `L3` + grip depth `l_g`) at `joint1 + joint2 + joint3` (all in
the radial–Z plane); the base azimuth fans the result around `+Z`. The fixed
vertical link `L0` adds a constant height offset. Useful identity used by the
IK (with `m_g` and `delta_g` from the section above):

```
d² = r² + (z - L0)² = L1² + m_g² + 2·L1·m_g·cos(joint2 + delta_g)
```

so the pose with `joint2 + delta_g = 0` reaches `L1 + m_g` from the joint1
pivot, and the pose with `joint2 + delta_g = ±180°` reaches `|L1 − m_g|`. Because
`m_g` depends on `joint3`, that maximum is itself a function of `joint3`.

The full finger geometry for a pose is available separately:

```text
gp = gripper_pose(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG, opening=None)
# -> FingerGripper
gp.joint1        # joint1 position (CartesianPoint)
gp.joint2        # joint2 position (CartesianPoint)
gp.joint3        # wrist joint position (end of L2)
gp.gripper_mount # gripper mount (end of L3)
gp.axis          # unit tool-axis direction
gp.spread        # finger spread off the axis [deg]
gp.fingertips    # 3 CartesianPoint tips (last point of each path)
gp.grip_point    # on-axis grasp centre (the FK result) — constant across openings
gp.finger_paths  # 3 polylines: each rigid finger's centreline, base -> tip
```

---

## 7. Inverse Kinematics (IK)

```text
inverse_kinematics(x, y, z, joint2_up=True, config=DEFAULT_CONFIG, opening=None, joint3=-60.0)
# -> JointAngles [deg]; joint3 may also be the string "auto" or "down"
```

The target `(x, y, z)` is the **grip point** (object centre).

1. **Base** (separable, single world-frame joint):

   ```
   theta_base = atan2(y, x)
   ```

2. **Collapse to 2-D**: `r = sqrt(x² + y²)`, `height = z - L0`, `d = sqrt(r² + height²)`.
   With the wrist stem AND the grip depth folded into an effective
   forearm `m_g = |L2 + (L3 + l_g)·e^(i·joint3)|`,
   `delta_g = arg(L2 + (L3 + l_g)·e^(i·joint3))`, the grip point behaves like a
   plain 2-link arm with links `(L1, m_g)`, so if `d` is outside
   `[|L1 − m_g|, L1 + m_g]`, raise `ValueError` — the target is
   geometrically unreachable.

3. **Effective elbow angle `te = joint2 + delta_g` — law of cosines.** From
   the FK identity:

   ```
   cos(te) = (d² − L1² − m_g²) / (2·L1·m_g)
   te      = ± acos(...)
   ```

   `cos(φ) = cos(−φ)` gives **two** solutions — that is *why* a joint2-up
   and a joint2-down configuration both reach the same point.

4. **Joint 1 — triangle trigonometry:**

   ```
   psi   = atan2(z, r)                                   # target bearing
   gamma = acos((L1² + d² − m_g²) / (2·L1·d))           # angle at joint 1
   ```

   | Configuration | `joint1` | `te` | physical `joint2` |
   | --- | --- | --- | --- |
   | joint2-up   | `psi + gamma` | `−acos(...)` | `te − delta_g` |
   | joint2-down | `psi − gamma` | `+acos(...)` | `te − delta_g` |

   These sign pairings are the **only** combinations that make
   `L1·e^{iθs} + m_g·e^{i(θs+θe+δg)}` point at the target.

> **Why `joint2` only depends on distance:** from step 3, `cos(te)` uses
> only `d`, `L1` and `m_g` — never the target's direction. The elbow bend is
> set purely by *how far* the target is from the base (plus the constant
> `delta_g`); the direction is handled by `theta_base` + `joint1`. So two
> targets at the same distance (e.g. `(400, 0, 0)` and `(0, 400, 0)`, both
> `d = 400`) return the **same** `joint1`/`joint2` (`joint1 ≈ 51.394°`,
> `joint2 ≈ −97.636°` at the default opening) and differ only in `theta_base`
> (`0°` vs `90°`). To see
> `joint2` change, change the distance: `(150, 0, 0)`, `(200, 0, 0)`,
> `(300, 0, 0)`, `(400, 0, 0)`, `(500, 0, 0)` give `≈ −132.03°`,
> `≈ −125.82°`, `≈ −112.30°`, `≈ −97.64°`, `≈ −81.60°` respectively. (Distances
> beyond the ≈ 839 mm outer bound at the default wrist, such as `(900, 0, 0)`, are unreachable.)

### The `"down"` mode

The five steps above all assume `joint3` is a free wrist angle **relative to L2**, which
is why `m_g` and `delta_g` vary and have to be recomputed per candidate. `joint3="down"`
replaces that with an *absolute* constraint: the stem plus grip depth points straight
down in the world frame, so

```
joint1 + joint2 + joint3 == -90°      (STEM_DOWN_ANGLE_DEG, for every pose)
```

Two things follow.

**It is not `joint3=-90`.** A numeric `joint3` rotates the wrist off the forearm, so
`joint3=-90` means "forearm bent, wrist hanging off it" and the stem is *not* vertical.
`"down"` instead chooses whatever `joint3` makes the total come out at −90°.

**It stays closed-form.** The stem is rigid, so with its direction pinned its
contribution to the grip point is a known constant vector. Peel that vector off the
target and what is left is a plain two-link `(L1, L2)` problem — steps 2 and 3 above
with `m_g → L2` and `delta_g → 0` — after which `joint3` is recovered from the
constraint rather than chosen:

```
stem  = -90°
d     = hypot(x, y - L0·0, ...)   # measured on the peeled-off point
joint1, joint2 = two-link solve    # joint2_up picks the branch
joint3 = stem - joint1 - joint2
```

So there is no search, and both elbow branches remain available.

The cost is reach. The peeled-off point must land inside `[|L1 − L2|, L1 + L2]`
= `[50, 550] mm`, **and** the recovered pose must still satisfy the joint limits. About
half of the targets reachable with a free wrist have no straight-down solution, and the
`ValueError` says so:

```python
try:
    ra.inverse_kinematics(0, 0, -40, joint3="down")
except ValueError as exc:
    print(exc)
# Target is unreachable with a straight-down wrist: after removing the stem offset the
# links must span 64.049 mm, outside [100.000, 700.000] mm.
# Use another joint3 value (or "auto") for this target.
```

Because each joint is wrapped independently, compare the identity modulo a full turn
rather than with `==`:

```python
a = ra.inverse_kinematics(300, 120, 150, joint3="down", opening=1.0)
sum_ = a.joint1 + a.joint2 + a.joint3
assert abs((sum_ + 90.0) % 360.0) < 1e-9      # True
```

`mind/verify_kinematics_down.py` verifies the stem geometrically — by forward
kinematics of the solved pose, not by trusting the solver's own angle arithmetic — for
both copies, across every elbow branch and all three gripper openings.

### Error behavior

IK raises a clear `ValueError` (never NaN/`None`) for targets that are:
outside the workspace, degenerate (on the base axis), or numerically
invalid (non-finite input). An unrecognised `joint3` **string** is also rejected, so a
typo like `"vertical"` fails loudly instead of being read as a wrist angle.

---

## 8. Workspace and joint limits — two separate ideas

**Geometric reachability** (`is_reachable`): the target is reachable iff

```
|L1 − m_g| ≤ d ≤ L1 + m_g,      d = sqrt( (sqrt(x²+y²))² + (z - L0)² )
```

where `L0` is the fixed vertical base link height and `m_g = |L2 + (L3 + l_g)·e^(i·joint3)|`
is the effective forearm — the wrist stem **and** the grip depth folded in, measured
along the direction set by `joint3`.

**This is why there is no single "reach" number.** `m_g` varies with `joint3`, so the
envelope is a family of shells — one per `joint3`, independent of the gripper opening.
Measured from the joint1 pivot, in mm:

| `joint3` | reach shell |
|---|---|
| ±90° | 37.18 … 762.82 |
| ±75° | 4.13 … 804.13 |
| ±60° | 39.15 … 839.15 |
| ±30° | 87.51 … 887.51 |
| **0°** | **104.05 … 904.05** |

The envelope is symmetric in the sign of `joint3` because `m_g` depends on
`cos(joint3)`. **Maximum reach is 904.05 mm, at `joint3 = 0°`** —
*not* at the `-60°` default, and *not* the `~606 mm` figure older revisions of this
document quote. That older number came from a half-length gripper model
(`l_g = FINGER_LENGTH/2 · cos(spread)`), which the code does not use; see
[issue 10](../README.md#10-sub-documentation-was-stale--corrected-in-this-release) in the root README.

`is_reachable` takes `joint3` as an explicit argument (default `-60°`) and shares its
bounds with the IK via `_reach_bounds_with_joint3`, so the two checks agree by
construction. Its `opening` argument is kept for API stability but no longer changes
the bounds. An obsolete helper that pinned the wrist to `WRIST_STEM_ANGLE` was removed;
the reach calculation now has one live, `joint3`-aware path.

> **Near-full reach, both ways:** at `joint3 ≈ ±76.60°` the effective forearm satisfies
> `m_g = L1 = 400 mm` exactly, so the inner bound `|L1 − m_g|` collapses to **0.00 mm**
> — the arm can reach its own base. Between roughly ±76.6° the shell is wide open at both
> ends; outside it, `m_g < L1` and the inner bound grows again. (Because `l_g` is
> constant, this no longer shifts when the gripper opens.)

**Joint-limit reachability** (`check_joint_limits`): whether the *physical*
arm can realize a solution. Limits are configuration parameters (degrees),
kept separate from the IK math so they can be tuned when the real arm is
built:

```text
L0 = 100.0                # mm  fixed vertical base link (base -> joint1)
L1 = 400.0                # mm  link 1 (joint1 -> joint2), upper arm
L2 = 300.0                # mm  link 2 (joint2 -> wrist mount), forearm
L3 = 80.0                 # mm  wrist stem (wrist mount -> gripper mount)
WRIST_STEM_ANGLE = -60.0  # deg  fixed stem angle phi relative to the forearm
GRIPPER_FINGERS = 3       # number of gripper fingers (tripod)
FINGER_LENGTH = 150.0     # mm  finger ARC length (Lf, one rigid curled bar)
FINGER_OPENING_DEG = 30.0 # deg  max swing of a finger off the tool axis
FINGER_CURL_DEG = 60.0    # deg  fixed inward curl of the rigid finger
GRIP_DEPTH = 124.05       # mm  constant on-axis grasp depth (= R_arc·sin(curl))
GRIPPER_OPENING = 1.0     # 0 = closed (tips meet on axis) .. 1 = open
JOINT_LIMITS = {
    "base":     (-180.0, 180.0),   # full signed yaw range
    "joint1":   (0.0, 180.0),      # link-1 angle from horizontal outward
    "joint2":   (-150.0, 150.0),   # relative link-2 angle
    "joint3":   (-150.0, 150.0),   # wrist angle relative to the forearm
}
```

Because the stem + grip depth is a rigid on-axis extension (fixed at whatever `joint3`
you passed), the kinematics reduce to an effective 2-link arm `(L1, m_g, delta_g)` while
the elbow (joint 2) physically stays at the end of `L2`.

A point can be geometrically reachable yet have no solution inside the
joint limits.

---

## 9. FK → IK → FK verification

```text
verify_fk_ik(n_samples=20, config=DEFAULT_CONFIG, opening=None)
```

For each random sample (sampled inside the joint limits) it:

1. computes the grip point `(x, y, z)` with FK,
2. solves IK in **both** joint2 modes,
3. re-runs FK on the IK result and compares positions,

and checks that FK(IK(x,y,z)) reproduces the original target within
`POSITION_TOLERANCE_MM = 0.1`. Exact *angle* recovery is additionally
asserted for front-facing poses (see the signed-`r` note above).

Example output:

```
========================================================
FK -> IK -> FK verification: PASS
Position error: 0.000000 mm  (tolerance 0.1 mm)
Max angle round-trip error: 0.000000 deg
Samples: 40/40 passed
========================================================
```

---

## 10. Visualization

`plot_robot(angles, config=..., target=None, opening=None)` renders the arm in
its own window; the tkinter GUI embeds the same picture via the shared
`draw_robot(ax, angles, ...)` primitive (both draw with identical
formulas/artists). The drawing includes:

- a visual-only, filled circular base platform of 50 mm radius in the XY
  plane at the origin; it is a drawing aid and does not alter the kinematic model,
- base, joint-1, joint-2, and wrist markers,
- the robot links (drawn from the FK joint positions, identical formulas),
- **the `GRIPPER_FINGERS` finger segments** from the wrist (cyan) up to each
  fingertip,
- global X (red) / Y (green) / Z (blue) axes,
- an optional thin dotted red connector from the wrist to the IK target when one is supplied.

Two entry points in the CLI:

- **Mode A — joint angles → robot:** option `4`.
- **Mode B — target XYZ → robot:** option `5` (runs IK, then plots the
  resulting pose *and* the target so you can see the match by eye).

In the **GUI**, `draw_robot` is fed by a `FigureCanvasTkAgg` in the
Visualization tab, whose toolbar lets you rotate/zoom the 3D view.

**Smooth transitions** add one more drawing mode:

- `interpolate_joints(from, to, frames=60)` — pure Python, returns the eased
  joint-space poses (`from` == first, `to` == last, `theta_base` via the
  shortest angular path). Reusable anywhere (a controller, the GUI, tests).
- `interpolate_opening(from_opening, to_opening, frames=60)` — the grip
  opening on the *same* ease curve, one value per frame, so the fingers move
  in step with the joints. Pass the same `frames` as `interpolate_joints`.
- `plot_animation(from, to, ..., from_opening=None)` (CLI option `7`) — a
  matplotlib `FuncAnimation` that steps `draw_robot` through those poses in
  its own window, keeping the optional dotted red line to the target visible so the hand is
  seen landing on it. Pass `from_opening` to ramp the grip instead of holding it.
- The GUI's **`Animate`** button drives the same interpolation with a tkinter
  timer (`root.after`), so both faces render identical motion.
- Both animations pre-compute `animation_limits(...)` over the whole
  trajectory and pin the 3D axes to that one frame, so the view stays
  rock-steady while the arm moves (the axes never zoom/pan per frame). The
  limits also accept a per-frame opening sequence, so an opening trajectory is
  enclosed too.

---

## 11. Implementation notes

- Math is always in **radians** internally; degree wrappers do the
  conversion.
- `acos` inputs are clamped to `[−1, 1]` to absorb floating-point error at
  the workspace boundary.
- All configurable values live in ONE configuration block at the top of
  `robot_arm.py` (`L1`, `L2`, `L3`, `WRIST_STEM_ANGLE`, `GRIPPER_FINGERS`,
  `FINGER_LENGTH`, `FINGER_OPENING_DEG`, `FINGER_CURL_DEG`, `GRIPPER_OPENING`,
  `JOINT_LIMITS`, tolerances).
- The gripper is purely **kinematic**: the fingers are drawn as rigid curled
  talons and their on-axis grip point is the end-effector for FK/IK/reachability,
  but the opening is a tool parameter, not a servo degree of freedom, and it does
  not affect the kinematics. Treating the stem + grip depth `(L3 + l_g)` as a
  fixed-angle on-axis extension keeps the
  planar solver closed-form: `L2 + (L3 + l_g)·e^(i·joint3) = m_g·e^(i·delta_g)`
  (`m_g ≈ 439.15 mm`, `delta_g ≈ −23.73°`, at `joint3 = −60°`). The physical elbow
  angle is recovered as `joint2 = te − delta_g`.
- Internal structure of the file (in order): imports → configuration →
  data structures (incl. `FingerGripper`) → math utilities → FK (+
  `gripper_pose`/`finger_tips`) → IK → workspace validation →
  joint-limit validation → FK/IK verification → visualization (`draw_robot`
  + `plot_robot`) → `RobotArm` facade → CLI → `main()`.
- `robot_arm_gui.py` wraps the core in a tkinter notebook (FK, IK,
  Reachability, Verification, Visualization) and reuses `draw_robot` so the
  embedded 3D view is identical to the CLI plot.

---

## 12. Everything explained — concepts, math and full API reference

A plain-English companion to the formulas in sections 5–8. Read this top to
bottom to understand the *why* behind every number in the code.

### 12.1 What this project is

`DUM-E` is a **simulator of a 3-joint robot arm** that solves the two
classic robot questions:

| Question | Kinematics | Answer |
| --- | --- | --- |
| *If I set the joints this way, where is the hand?* | **forward** (FK) | one `(x, y, z)` — unique and cheap |
| *To reach this point, which joint angles do I need?* | **inverse** (IK) | two elbow poses (`joint2-up` / `joint2-down`) |

It is pure mathematics + a 3D plot: no Arduino, no servos, no hardware. It is
the *brain* that a future controller would call. Everything lives in
`robot_arm.py` (math core + CLI), with `robot_arm_gui.py` (tkinter UI) on top.

### 12.2 The physical model — four joints, three links, one stem, three fingers

```
Base ──(L0=100 mm fixed)── joint1 ──(L1=400 mm)── joint2 ──(L2=300 mm)── wrist mount
                                                                       │  L3 (80 mm), kinked −60° off L2
                                                                      ▼
                                                                  gripper mount ── 3 fingers × 150 mm, 120° apart
```

- **Base**: the only joint attached to the world. Rotates the whole arm
  around vertical **+Z**. Range −180°..180°. The fixed vertical link `L0 = 100 mm`
  connects the base to `joint1` and cannot tilt.
- **`joint1`** (shoulder / L1 tilt): lifts/lowers link 1 in a vertical plane.
  Measured from horizontal outward; `joint1 = 0` → arm horizontal, `90°` → straight up.
  Range 0°..180°, shared with the command brain via `robot_contract.py`.
- **`joint2`** (elbow / L2 tilt): bends link 2 *relative to* link 1. `0` → arm straight.
  Range ±150°.
- **Stem `L3`**: a rigid 80 mm segment connected by the commandable `joint3`,
  whose default/home angle is `-60°` relative to link 2. The stem runs from the
  **wrist mount** (end of L2) to the **gripper mount** (end of L3).
- **Fingers**: `GRIPPER_FINGERS = 3` rigid **curled talons**, each `FINGER_LENGTH = 150 mm`
  of arc length curling inward by `FINGER_CURL_DEG = 60°`, mounted on a ring of radius
  `R_base = 71.62 mm` around the tool axis and spaced 120° apart. The opening
  `spread = GRIPPER_OPENING × 30°` only **swings** the whole rigid finger about its base:
  `0` (closed) → the tips meet ON the axis, `1` (open) → they fan out to 30°. There is
  no joint along a finger and the curvature never changes.
- **Palm**: a visual flange (spokes + base-ring outline) bridges the wrist tip
  (`gripper_mount`, on the axis) to each finger base on that `R_base` ring, so the claw
  reads as attached to the arm. It is drawn only — it adds no joint and no kinematics.

**So the arm has exactly 4 degrees of freedom**: 4 angles (base, joint1, joint2, joint3) fully determine
every point (joint1, joint2, joint3, gripper mount, fingertips, object centre). The fingers
have no motors and never change the reach — their open/close is a *tool setting*, not a
degree of freedom — but it is still animated: during a transition the opening eases
alongside the joints (see §4.4 and `interpolate_opening`).

### 12.3 Units and conventions (memorize these)

- Angles enter the public API in **degrees**, exit in **degrees**.
- Positions are **millimetres** in a right-handed frame: **+Z up**, **+X
  forward** at `theta_base = 0`.
- Internally every angle is **radians**; the degree↔radian conversion happens
  once at the door (`_to_rad`/`_to_deg`).
- An angle is either **absolute** (in world/plane coordinates, e.g.
  `joint1`, `joint1 + joint2`, `joint1 + joint2 + phi`) or **relative**
  (`joint2` is measured off link 1, not off horizontal). Getting this wrong
  is the classic source of bugs.

### 12.4 Forward kinematics — "angles in, point out"

FK just **adds the link vectors**:

1. The fixed vertical link `L0` places `joint1` at `(0, 0, L0)`.
2. Link 1 points at `joint1`, link 2 at `joint1 + joint2`, the stem+grip
   depth at `joint1 + joint2 + phi` (all in the radial–Z plane).
3. Add their horizontal parts → signed radius `r`; add their vertical parts
   → height `z = L0 + ...`. (That is the big three-term equations in section 6.)
4. The base azimuth fans that planar point around +Z: `x = r·cos(θ)`,
   `y = r·sin(θ)`.

Result: the **grip point** — the object centre, not the wrist. The 3D
finger positions are a bonus built on top (see §12.5).

### 12.5 The grip point and the effective-forearm trick (the key idea)

Two rigid vectors whose angle between them is **constant** always add to a
single vector whose **length** and **angle** are also constant. The last two
vectors of the arm — the forearm `L2` and the on-axis extension
`L3 + l_g` (stem + grip depth) — have exactly that fixed angle between them (it
is `joint3`, constant *within a pose*), so they collapse into one vector:

```
L2·e^(i·(j1+j2))  +  (L3 + l_g)·e^(i·(j1+j2+phi))
   =  m_g·e^(i·(j1+j2+delta_g))
```

The arm is therefore *equivalent to a plain 2-link arm* with link 1 = `L1`
and link 2 = the effective forearm `m_g` (stem offset by `delta_g`). This is
**why the IK never needs to iterate** — it is closed-form trigonometry
(`atan2`, `acos`) on that 2-link arm. Constants:

- Stem only (`l_g = 0`): `m ≈ 346.99 mm`, `delta ≈ −11.52°`.
- Grip point (`l_g = 124.05 mm`): `m_g ≈ 439.15 mm`, `delta_g ≈ −23.73°`.

Both are evaluated at `joint3 = −60°`. The stem-only figures are independent of the
gripper; the grip-point figures vary with `joint3` but **not** with the opening, because
`l_g` is constant.

The grip point `G = gripper_mount + l_g·axis` lies ON the tool axis at the
**constant** depth `l_g = GRIP_DEPTH = 124.05 mm`; on-axis points fold into the same
planar model — which is exactly why a **3-finger gripper adds zero mathematical
complexity**. Swinging the fingers does not move `G` at all, so the opening never
changes the reach.

### 12.6 Inverse kinematics — "point in, angles out"

IK walks the 2-link arm *backwards*:

1. **Base**: `theta_base = atan2(y, x)` — the azimuth is trivially separable
   because the base is the only world-frame joint.
2. **Collapse 3D → 2D**: `r = sqrt(x²+y²)`, `height = z - L0`, `d = sqrt(r²+height²)`. Now it is a
   2-link planar problem with links `(L1, m_g)` and target distance `d`.
3. **Elbow by the law of cosines**:
   `cos(te) = (d² − L1² − m_g²)/(2·L1·m_g)`, `te = ±acos(...)`.
   Because `cos(θ) = cos(−θ)` there are **two** elbows that satisfy it — one
   folded above the base→target line (`joint2-up`), one below
   (`joint2-down`). That is the whole reason the "up/down" choice exists and
   why both options place the hand at the *same* Cartesian point.
4. **Shoulder by triangle angles**: `psi = atan2(z, r)` (target bearing),
   `gamma = acos((L1²+d²−m_g²)/(2·L1·d))` (triangle angle at the shoulder).
   The two pairings are forced (not arbitrary):
   `joint2-up → joint1 = psi + gamma, te = −acos`, `joint2-down → joint1 =
   psi − gamma, te = +acos`.
5. **Recover the physical elbow** (undo the stem offset):
   `joint2 = te − delta_g`.

### 12.7 Why `joint2` only depends on *distance*

Look at step 3: the formula for `cos(te)` contains only `d`, `L1`, `m_g` —
**never** the direction of the target. The depth of the elbow bend is set
purely by *how far* the target is; the direction is handled by `theta_base`
and `joint1`. So `(400,0,0)` and `(0,400,0)` (both `d = 400`) give the same
`joint1 ≈ 51.394°` and `joint2 ≈ −97.636°`, differing only in `theta_base`
(`0°` vs `90°`). To alter `joint2`, alter `d`.

### 12.8 Reachability vs joint limits — two different "can I make it?"

| Check | Question | Purview |
| --- | --- | --- |
| `is_reachable` | Is the point inside the geometric shell `[|L1−m_g|, L1+m_g]`? | geometry only |
| `check_joint_limits` | Does the *physical* arm allow those angles? | servo ranges |

A point can easily be reachable yet unachievable (both IK solutions outside
the servo limits), or the other way around. That is why they are separate
functions with separate purposes — and why the IK is deliberately allowed to
return angles outside the limits (it answers the *geometric* question; you
validate the *physical* one yourself).

### 12.9 The signed-`r` subtlety (the arm "behind" the base)

`r` is a **signed projection**, not a distance. If the elbow folds far enough
past the vertical axis, the hand's horizontal projection goes negative — the
hand is literally on the far side of the base. The true azimuth is then
`theta_base + 180°`. Because IK always returns a **front-facing** pose
(`theta_base = atan2(y,x)`), it *still reaches the point* but it will not
reproduce the original angles of a backward-folded pose. The verification
harness accounts for this (position must round-trip for ALL samples; exact
angles only for front-facing ones).

### 12.10 Verification — what PASS actually proves

`verify_fk_ik(n_samples, opening)` does, for each random angle triple inside
the limits:

```
angles ─FK→ (x,y,z) ─IK(up,down)→ 2 angle triples ─FK→ (x',y',z')
```

`PASS` means `(x',y',z') == (x,y,z)` within `0.1 mm` for **both** elbow
solutions — i.e. the forward and inverse implementations are mathematically
consistent inverses across the whole workspace, not just on hand-picked
examples. The angle round-trip check additionally confirms the *same* elbow
pose is recovered for front-facing inputs.

### 12.11 The three interfaces (one brain, three faces)

```
                     ┌─────────────────────────────┐
   python robot_arm.py  │  robot_arm.py (math core)  │
     (text menu)        │  FK · IK · reach · limits  │  python robot_arm_gui.py
   cli() ───────────────►  · verify · plot            │◄────────────── (tkinter app)
                     │  RobotArm facade             │   5 tabs, same draw_robot
                     └─────────────────────────────┘
                                 │
                          draw_robot(ax, ...)    ← one renderer, three places:
                          plot_robot() (CLI window)
                          FigureCanvasTkAgg (GUI)
```

The GUI's five tabs map exactly onto the core: **FK**, **IK**,
**Reachability**, **Verification**, **Visualization**. No duplicated math — a
"Visualize" button in the FK/IK tabs simply pre-fills the Visualization tab
and calls the same `draw_robot` the CLI uses.

### 12.12 Complete public API reference

**Data types** (`robot_arm.py`):

| Type | Fields | Notes |
| --- | --- | --- |
| `CartesianPoint` | `x`, `y`, `z` (mm) | plus `.coords() → (x, y, z)` |
| `JointAngles` | `theta_base`, `joint1`, `joint2`, `joint3` (deg) | the output of IK |
| `FingerGripper` | `joint1`, `joint2`, `joint3`, `gripper_mount`, `axis`, `spread`, `fingertips`, `grip_point`, `opening`, `finger_paths` | produced by `gripper_pose()` |
| `RobotConfig` | `l0`, `l1`, `l2`, `l3`, `wrist_stem_angle`, `gripper_fingers`, `finger_length`, `finger_opening_deg`, `finger_curl_deg`, `gripper_opening`, `joint_limits` | one object = one arm |

**Module functions** (all take `config: RobotConfig = DEFAULT_CONFIG` and,
where noted, an optional `opening`):

| Function | Signature | Returns |
| --- | --- | --- |
| `forward_kinematics` | `(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG, opening=None)` | grip point `CartesianPoint` |
| `gripper_pose` | `(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG, opening=None)` | full `FingerGripper` geometry |
| `finger_tips` | `(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG, opening=None)` | `tuple` of 3 `CartesianPoint` |
| `inverse_kinematics` | `(x, y, z, joint2_up=True, config=DEFAULT_CONFIG, opening=None, joint3=-60.0)` | `JointAngles` (deg); `joint3` may be `"auto"` or `"down"` |
| `is_reachable` | `(x, y, z, config=DEFAULT_CONFIG, joint3=-60.0, opening=None)` | `bool` (geometric) |
| `check_joint_limits` | `(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG)` | `bool` (physical) |
| `requires_joint_limits` | `(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG)` | `JointAngles` or raises `ValueError` |
| `verify_fk_ik` | `(n_samples=20, opening=None)` | `(max_pos_err, passed, total)` |
| `draw_robot` | `(ax, angles, target=None, opening=None, fixed_limits=None)` | draws into a 3D `Axes` |
| `plot_robot` | `(angles, target=None, opening=None)` | opens its own window |
| `interpolate_joints` | `(from_angles, to_angles, frames=60)` | eased joint-space poses (tuple) |
| `interpolate_opening` | `(from_opening, to_opening, frames=60)` | eased grip openings, one per frame (tuple) |
| `animation_limits` | `(poses, opening=None, target=None)` | one equal-aspect box; `opening` may be a scalar or one value per pose |
| `plot_animation` | `(from_angles, to_angles, target=None, opening=None, frames=60, interval=30, from_opening=None)` | animates in its own window; ramps the grip when `from_opening` is given |
| `cli` / `main` | — | the text menu |

**`RobotArm` facade** — a convenience class bound to one config. **Its kinematic methods
are currently broken** (see the facade note in §3 for which ones); the module-level
functions are the supported path:

```text
arm = ra.RobotArm()                      # default 400/300/80 arm
arm.l1, arm.l2, arm.l3, arm.stem_angle   # read-only geometry
arm.inverse_kinematics(300, 120, 150, joint2_up=True)   # works
arm.plot(angles, target=p)               # works
arm.verify(n_samples=50)                 # works

# these four raise TypeError as shipped — they omit joint3:
#   arm.forward_kinematics(theta_base, joint1, joint2, opening=None)
#   arm.gripper_pose(theta_base, joint1, joint2, opening=None)
#   arm.check_joint_limits(theta_base, joint1, joint2)
#   arm.is_reachable(x, y, z, opening=None)
```

### 12.13 Configuration and derived values (default arm)

Configurable (one block at the top of `robot_arm.py`):

| Setting | Default | Meaning |
| --- | --- | --- |
| `L0` | 100 mm | fixed base column |
| `L1` | 400 mm | upper arm |
| `L2` | 300 mm | forearm |
| `L3` | 80 mm | wrist stem |
| `WRIST_STEM_ANGLE` | −60° | legacy value used by `_grip_parameters` for reporting; use the `joint3` argument for live FK/IK |
| `GRIPPER_FINGERS` | 3 | finger count (tripod) |
| `FINGER_LENGTH` (`Lf`) | 150 mm | finger ARC length (one rigid curled bar) |
| `FINGER_OPENING_DEG` | 30° | max swing of a finger off the tool axis |
| `FINGER_CURL_DEG` | 60° | fixed inward curl of the rigid finger |
| `GRIPPER_OPENING` | 1.0 | default opening (0 closed .. 1 open) |
| `JOINT_LIMITS` | base ±180°, j1 0..180°, j2/j3 ±150° | shared ranges from `robot_contract.py` |
| `POSITION_TOLERANCE_MM` | 0.1 | verification position bar |
| `REACH_EPSILON_MM` / `MIN_D_SHARE_LIMIT` | 1e-9 | numerical slack/guards |

Derived (via `_grip_parameters`; these are the `joint3 = −60°` values, and none of them
now depends on the opening):

| Quantity | Value |
| --- | --- |
| grip depth `l_g` | 124.05 mm (constant) |
| effective forearm `m_g` | 439.15 mm |
| forearm offset `delta_g` | −23.73° |
| reach `[min, max]` at joint3 −60° | [39.15, 839.15] mm |
| reach `[min, max]` at joint3 0° (max) | [104.05, 904.05] mm |

Earlier revisions of this table had **two** columns, one per opening, because
`l_g = FINGER_LENGTH·cos(spread)` made the grip depth — and therefore the reach — depend
on the gripper opening. The fingers are now rigid curled talons with the constant
`l_g = GRIP_DEPTH`, so the family collapses to one value per `joint3`. Reach still
depends on `joint3`, so it is not one pair of numbers at all — see
[§8](#8-workspace-and-joint-limits--two-separate-ideas).

Note that `L1` (400 mm) and the effective forearm `m_g` (≈ 439.1 mm) are close but not
equal, giving a small but non-zero inner bound at the default `joint3`. They become
*exactly* equal at `joint3 ≈ ±76.60°`, where the inner bound reaches 0.00 mm.

### 12.14 Common questions

- **Why is `joint2` the same for every target at the same distance?** The
  elbow formula depends only on `d` (§12.7).
- **Which elbow should I pick, up or down?** Whichever stays inside the joint
  limits and avoids obstacles; they both reach the same point.
- **Does opening the gripper change the reach?** No. The grasp point is a
  constant on-axis depth (`l_g = GRIP_DEPTH = 124.05 mm`), so swinging the fingers
  never moves `G` or the effective forearm `m_g`. (Earlier revisions had `l_g`
  fall as the fingers opened, so closing the gripper used to lengthen the reach.)
- **Why do the demo numbers look "too perfect" (1e-14 errors)?** The FK and IK
  are consistent closed-form inverses; the residual is floating-point noise,
  and the 0.1 mm tolerance is generous by design.
- **Why do fingers exist if they add no DOF?** They make the end-effector the
  *object centre* (grip point) instead of an arbitrary wrist marker, which is
  the point a real gripper aims at — and they make the visualization legible.
- **Can I change the arm?** Yes — pass a custom `RobotConfig`
  (`ra.RobotConfig(l1=150.0, ...)`), or edit the constants block. Every
  function accepts a `config=` argument.
- **Where does an animation start from?** From the last pose that was drawn.
  In the CLI that is whatever option `1`/`4`/`5` or the previous animation
  ended at (initially `HOME_ANGLES = (0, 90, 0)`); in the GUI it is the
  current picture on the Visualization tab. The grip opening is treated the
  same way: a transition eases from the last drawn opening to the target one.

### 12.15 Glossary

- **DOF** — degree of freedom; an independently controllable joint (here: 4 - `theta_base`, `joint1`, `joint2`,
`joint3`). The gripper opening is *not* a DOF; it is a tool parameter that only
swings the rigid fingers and does **not** change the reach — though its motion is
still eased during an animation.
- **FK / IK** — forward / inverse kinematics.
- **Grip point** — on-axis grasp centre (the constant point IK aims at); the end-effector.
- **Effective forearm `m_g`** — the single vector that `L2 + (L3+l_g)` at the
  fixed stem angle collapses to; turns the arm into a 2-link problem.
- **`joint2-up` / `joint2-down`** — the two elbow configurations IK can return.
- **Signed `r`** — horizontal projection that can go negative when the hand
  passes behind the base axis.
- **Reachability vs joint limits** — geometric possibility vs physical
  realizability.
- **`draw_robot`** — the single matplotlib renderer shared by the CLI window
  and the GUI.
- **Easing** — the ease-in/out curve applied to animation progress, so the
  arm accelerates out of the start pose and decelerates into the target
  instead of moving at constant speed.
- **Joint-space interpolation** — per-joint linear motion between two poses
  (what real servos approximate), the opposite of a Cartesian straight-line
  hand path.