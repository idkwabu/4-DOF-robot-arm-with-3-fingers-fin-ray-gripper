# DUM-E — 4-DOF Robot Arm Kinematics Core

A single-file mathematics + visualization layer for a 4-DOF robot arm
(`Base → fixed L0 → joint1/L1 → joint2/L2 → joint3/wrist mount → L3 →
gripper mount → Fingers`). The model is a fixed vertical `L0 = 50 mm` base
link, a `300 mm` L1 upper link, a `250 mm` L2 forearm, and a `70 mm` L3 wrist
stem. **The four DOF are `theta_base`, `joint1`, `joint2` and `joint3`** — `joint3`
is the wrist and is a real, commandable joint (`±150°`, default `-60°`), *not* a
fixed kink. The `WRIST_STEM_ANGLE = -60°` config field is a legacy constant that
survives only in the dead `_reach_bounds` helper and the `_grip_parameters`
reporting path; the live kinematics all use the `joint3` you pass in. L0 cannot
tilt; the base rotates about world `+Z`; joint1 and joint2 control radial-Z tilt
and have no independent spin. At the gripper mount sits a simulated **3-finger
gripper** (`GRIPPER_FINGERS = 3`): three straight `FINGER_LENGTH` segments
spaced 120° around the tool axis, whose on-axis **grip point** (the fingertip
centroid — where a gripped object sits) is the end-effector that FK/IK/
reachability use. The opening is configurable (`GRIPPER_OPENING`, 0 = closed
with the tips meeting on the axis, 1 = fully open).

> **Reach envelope:** the envelope is **not a single number** — it depends on
> `joint3`, because the effective forearm `m` is measured from `L2` along a
> direction set by that joint. Reachable radial-Z distances are measured from the
> joint1 pivot at `(0, 0, L0)`, not from the world origin. With the default
> `joint3 = -60°` and the gripper open the range is `21.97 … 621.97 mm`; the
> **maximum** over all `joint3` is `63.30 … 663.30 mm` at `joint3 = 0°`
> (`70.00 … 670.00 mm` with the gripper closed). See
> [§8](#8-workspace-and-joint-limits--two-separate-ideas) for the full table.

**Stage scope:** forward kinematics, inverse kinematics, workspace validation,
joint-limit validation, FK→IK→FK verification, a 3D visualization — plus
**smooth joint-space animations** between poses (CLI option `7` and the GUI
`Animate` button) — available both as a text CLI (`robot_arm.py`) and as a
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

Everything lives in two files:
- `robot_arm.py` — the kinematics core (import it as a library, run it as a CLI).
- `robot_arm_gui.py` — the tkinter desktop UI (imports `robot_arm`).

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
| `3` | Is a target reachable? (gripper opening affects the reach!) |
| `4` | Joint angles → 3D plot of the robot + fingers |
| `5` | Target `(x, y, z)` → IK → 3D plot of the robot + target marker |
| `6` | Run the FK→IK→FK verification suite |
| `7` | Smooth joint-space animation from the last drawn pose to an IK target |
| `8` | Exit |

Options `1`–`7` additionally ask for an optional **gripper opening** (0..1;
Enter keeps the config default `gripper_opening`); option `7` also asks for
the **animation frame count** (Enter keeps 30).

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
# Pass joint3="auto" to let the solver pick it.
angles = ra.inverse_kinematics(300.0, 120.0, 150.0, joint2_up=True)  # JointAngles

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

### Known bug: the `RobotArm` facade drops `joint3`

`RobotArm` wraps a config so you need not pass `config=` everywhere, but its kinematic
methods were **not updated when `joint3` was added** to the 4-DOF arm. As shipped,
**5 of its 7 public methods raise `TypeError`**:

| Method | Status |
| --- | --- |
| `RobotArm.forward_kinematics(theta_base, joint1, joint2, opening=None)` | **broken** — no `joint3`; the 4th positional is captured as `config` |
| `RobotArm.gripper_pose(...)` | **broken** — same cause |
| `RobotArm.check_joint_limits(...)` | **broken** — takes 4 positional args, so a `joint3` argument is rejected |
| `RobotArm.is_reachable(...)` | **broken** — forwards `joint3=None` into the geometry |
| `RobotArm.inverse_kinematics(...)` | works |
| `RobotArm.plot(...)` | works |
| `RobotArm.verify(...)` | works |

**Use the module-level functions in §12 above** — they take a `config` keyword and are
correct. `RobotArm.inverse_kinematics`, `.plot` and `.verify` are safe if you have a
`RobotArm` instance for other reasons. This is a source defect, not a documentation
choice; see [issue 1](../README.md#1-the-robotarm-facade-is-missing-joint3-and-passes-the-wrong-arity) in the root README.

---

## 4. How to use it — concrete walkthrough

### 4.1 CLI, end to end

From the project folder:

```bash
python robot_arm.py
```

The menu appears. **Option `1` — Forward Kinematics** asks for three joint
angles in degrees (then an optional gripper opening) and returns the **grip
point** plus every fingertip:

```
Select option: 1
  base yaw       (deg): 45
  joint1 / L1    (deg): 30
  joint2 / L2    (deg): -20
  gripper opening 0..1 (default 1, Enter = default):

  Grip point: x=399.460 mm, y=399.460 mm, z=173.204 mm  (opening 1.000)
  Finger 1 tip: x=408.299 mm, y=390.621 mm, z=173.204 mm
  Finger 2 tip: x=389.177 mm, y=398.015 mm, z=166.245 mm
  Finger 3 tip: x=400.904 mm, y=409.743 mm, z=180.162 mm
```

(For reference, the pose `45, 30, -20` with opening `0` — fingers closed on
 the axis — has its grip point at `x=400.982, y=400.982, z=170.638` mm.)


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
  joint1=  77.435 deg
  joint2= -96.480 deg
  within joint limits: yes
```

Try the *down* configuration for the same target — you get a different pose
(`joint1= -43.041 deg, joint2= 131.968 deg`) whose grip point is still exactly
`(300, 120, 150)`. Try option `3` with `(700, 0, 0)`: it is **UNREACHABLE** at the
default `joint3 = -60°`, because that is beyond `L1 + m = 621.97` mm from the joint1
pivot. (It is still unreachable at *every* `joint3`: the largest outer bound the arm has
is `663.30 mm`, at `joint3 = 0°`.) The inner bound at the default `joint3` is only
21.97 mm, so `(100, 0, 0)` is reachable geometrically from the `z = L0` pivot; check the
joint limits before relying on it.


**Option `5` — Visualize From XYZ Target** opens a matplotlib window
(3 fingers drawn in cyan, grip point as a small green wireframe octahedron).
Confirm the target (`red +`) sits exactly on the green grip-point marker: run
option `5` with the same `(300, 120, 150)` and check the recon-FK line:

```
  IK result: base=21.801, joint1=77.435, joint2=-96.480 (deg)
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
branch fixed for the whole move so the arm cannot flip mid-animation. Targets
outside the geometric workspace are rejected with an error.

**Option `8`** quits.

### 4.2 As a library / script

Create a file `demo.py` alongside `robot_arm.py`:

```python
"""Minimal DUM-E demo: FK, IK, workspace, limits, then a 3D plot."""
import robot_arm as ra

# Module-level functions are used throughout: the RobotArm facade's kinematic
# methods still drop joint3 and raise TypeError (see the facade note in section 3).


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

    # 4) Workspace and limits (opening and joint3 both change the reach)
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
FK(45, 30, -20) -> x=399.460 y=399.460 z=123.204
   spread=30.0 deg, 3 fingertips
   wrist = (389.619, 389.619, 139.789) mm
IK up   -> base= 21.801  joint1= 79.741  joint2= -93.021
IK down -> base= 21.801  joint1=-29.936  joint2= 123.060
   round-trip error: 8.53e-14 mm
   round-trip error: 6.51e-14 mm
reachable(300,120,150): True
reachable(700,0,0):     False
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
| **Reachability** | target (mm) + opening | reachable?, `d`, reach bounds for the current/closed/open gripper |
| **Verification** | `n_samples` + opening | FK→IK→FK PASS/FAIL suite output |
| **Visualization** | joint angles **or** XYZ target + opening | embedded 3D plot of the arm, fingers (cyan), grip point (green wireframe octahedron), target (red `+`); an `Animate` button plays the smooth joint-space transition from the current pose |

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
  it becomes `Stop` while running, so you can cancel mid-motion.
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
`170° → -170°` sweeps through `180°`, not through `0°`.

```
Select option: 7
  x (mm): 300
  y (mm): 120
  z (mm): 150
  joint2 pose [up/down] (up): up
  gripper opening 0..1 (default 1, Enter = default): 1
  animation frames (default 30, Enter = default):

  Animating joint-space transition:
    start  base=  45.000  joint1=  30.000  joint2= -20.000 deg   <- last drawn pose
    target base=  21.801  joint1=  79.741  joint2= -93.021 deg
```

A matplotlib window opens and the arm eases from the start pose to the
target (whose red `+` marker stays visible, so you see the hand land on it).
When it closes, the target pose becomes the new "last drawn" pose — run
option `7` again to chain another transition from where the arm just ended.
If the target pose is *identical* to the last drawn pose (e.g. you re-enter
the same target you just drew), option `7` says so instead of opening an
empty animation.

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
m     = |L2 + L3·e^(i·joint3)|        (≈ 291.38 mm at joint3 −60°)
delta = arg(L2 + L3·e^(i·joint3))     (≈ −12.01° at joint3 −60°)
```

**Gripper fingers & grip point:** `GRIPPER_FINGERS = 3` straight segments of
length `FINGER_LENGTH` (50 mm) start at the **gripper mount**, azimuthally
spaced 120° around the tool axis and each angled `spread` off it. The spread
follows the configurable opening:

```
spread   = GRIPPER_OPENING · FINGER_OPENING_DEG            (default 1 → 30°)
l_g      = FINGER_LENGTH · cos(spread)    # grip depth along the tool axis
```

The **grip point** `G = gripper_mount + l_g·axis` is the on-axis centroid of the
three fingertips — the point where a gripped object sits and the target that
FK/IK/reachability use. Because `G` sits on the tool axis, it folds into the
same planar picture: the on-axis extension `(L3 + l_g)` replaces `L3` above,
giving an *opening-dependent* effective forearm

```
a_g     = L3 + l_g                       # stem plus grip depth, rigid together
m_g     = |L2 + a_g·e^(i·phi)|            (≈ 321.97 mm @ joint3 −60°, opening 1)
delta_g = arg(L2 + a_g·e^(i·phi))         (≈ −17.74° @ joint3 −60°, opening 1)
```

Two things follow, and both matter:

**`phi` is `joint3`, not the `WRIST_STEM_ANGLE` constant.** The stem and the grip
depth rotate together with the wrist, so `m_g` and `delta_g` are functions of the
*commanded* `joint3` and of the opening. That is what makes the reach envelope in
[§8](#8-workspace-and-joint-limits--two-separate-ideas) joint3-dependent. The table
below is evaluated at the default `joint3 = −60°`; every `m_g` figure elsewhere in
this document assumes the same.

**The degree figures are** `delta_g ≈ −17.74°` open / `≈ −18.53°` closed.
(`delta_g` is naturally a radian phase, handled internally in radians.)

`opening = 0` (closed) pushes the grip point furthest (tips meet on the axis,
`l_g = FINGER_LENGTH = 50.00 mm`, `m_g ≈ 326.96 mm`); `opening = 1` pulls it back
(`l_g = 43.30 mm`, `m_g ≈ 321.97 mm`). So closing the gripper slightly *increases*
the reach (by ≈ 4.99 mm at the default lengths).

---

## 6. Forward Kinematics (FK)

```text
forward_kinematics(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG, opening=None)
# -> CartesianPoint(x, y, z) [mm]
```

Returns the **grip point** (on-axis centroid of the fingertips). Note `joint3` is a
required positional argument. With `l_g = FINGER_LENGTH·cos(spread)` and `phi` = the
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
gp.fingertips    # 3 CartesianPoint tips
gp.grip_point    # on-axis centroid (the FK result)
```

---

## 7. Inverse Kinematics (IK)

```text
inverse_kinematics(x, y, z, joint2_up=True, config=DEFAULT_CONFIG, opening=None, joint3=-60.0)
# -> JointAngles [deg]; joint3 may also be the string "auto"
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
> `d = 400`) return the **same** `joint1`/`joint2` (`joint1 ≈ 49.399°`,
> `joint2 ≈ −82.426°` at the default opening) and differ only in `theta_base`
> (`0°` vs `90°`). To see
> `joint2` change, change the distance: `(150, 0, 0)`, `(200, 0, 0)`,
> `(300, 0, 0)`, `(400, 0, 0)`, `(500, 0, 0)` give `≈ −136.36°`,
> `≈ −126.48°`, `≈ −105.68°`, `≈ −82.43°`, `≈ −53.87°` respectively. (Distances
> beyond the ≈ 606 mm outer bound, such as `(700, 0, 0)`, are unreachable.)

### Error behavior

IK raises a clear `ValueError` (never NaN/`None`) for targets that are:
outside the workspace, degenerate (on the base axis), or numerically
invalid (non-finite input).

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
envelope is a family of shells. Measured from the joint1 pivot, in mm:

| `joint3` | shell, gripper open (`opening=1`) | shell, gripper closed (`opening=0`) |
|---|---|---|
| ±90° | 25.52 … 574.48 | 22.69 … 577.31 |
| ±75° | 0.00 … 600.00 | 4.02 … 604.02 |
| ±60° | 21.97 … 621.97 | 26.96 … 626.96 |
| ±30° | 52.70 … 652.70 | 58.97 … 658.97 |
| **0°** | **63.30 … 663.30** | **70.00 … 670.00** |

The envelope is symmetric in the sign of `joint3` because `m_g` depends on
`cos(joint3)`. **Maximum reach is 663.30 mm open / 670.00 mm closed, at `joint3 = 0°`** —
*not* at the `-60°` default, and *not* the `~606 mm` figure older revisions of this
document quote. That older number came from a half-length gripper model
(`l_g = FINGER_LENGTH/2 · cos(spread)`), which the code does not use; see
[issue 10](../README.md#10-sub-documentation-was-stale--corrected-in-this-release) in the root README.

`is_reachable` takes `joint3` as an explicit argument (default `-60°`) and shares its
bounds with the IK via `_reach_bounds_with_joint3`, so the two checks agree by
construction. A private `_reach_bounds` helper that pins the wrist angle to the
`WRIST_STEM_ANGLE` constant also exists but is **dead code** — it is correct only when
`joint3` happens to equal that constant.

> **Near-full reach, both ways:** at `joint3 ≈ ±75°` the effective forearm satisfies
> `m_g = L1 = 300 mm` exactly, so the inner bound `|L1 − m_g|` collapses to **0.00 mm**
> — the arm can reach its own base. Between roughly ±75° the shell is wide open at both
> ends; outside it, `m_g < L1` and the inner bound grows again. Note that closing the
> gripper *raises* both bounds, so a target very near the base can become unreachable
> when the fingers close.

**Joint-limit reachability** (`check_joint_limits`): whether the *physical*
arm can realize a solution. Limits are configuration parameters (degrees),
kept separate from the IK math so they can be tuned when the real arm is
built:

```text
L0 = 50.0                 # mm  fixed vertical base link (base -> joint1)
L1 = 300.0                # mm  link 1 (joint1 -> joint2), upper arm
L2 = 250.0                # mm  link 2 (joint2 -> wrist mount), forearm
L3 = 70.0                 # mm  wrist stem (wrist mount -> gripper mount)
WRIST_STEM_ANGLE = -60.0  # deg  fixed stem angle phi relative to the forearm
GRIPPER_FINGERS = 3       # number of gripper fingers (tripod)
FINGER_LENGTH = 50.0      # mm  finger segment length
FINGER_OPENING_DEG = 30.0 # deg  max finger spread off the tool axis
GRIPPER_OPENING = 1.0     # 0 = closed (tips meet on axis) .. 1 = open
JOINT_LIMITS = {
    "base":     (-180.0, 180.0),   # full revolution
    "joint1":   (-90.0,  110.0),   # link-1 angle from horizontal outward
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

- base, joint-1, joint-2, and wrist markers,
- the robot links (drawn from the FK joint positions, identical formulas),
- **the `GRIPPER_FINGERS` finger segments** from the wrist (cyan) up to each
  fingertip, plus the grip point (a small green wireframe octahedron, == the FK
  result),
- global X (red) / Y (green) / Z (blue) axes,
- an optional red `+` target marker when an IK target is supplied.

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
- `plot_animation(from, to, ...)` (CLI option `7`) — a matplotlib
  `FuncAnimation` that steps `draw_robot` through those poses in its own
  window, keeping the optional red `+` target visible so the hand is seen
  landing on it.
- The GUI's **`Animate`** button drives the same interpolation with a tkinter
  timer (`root.after`), so both faces render identical motion.
- Both animations pre-compute `animation_limits(...)` over the whole
  trajectory and pin the 3D axes to that one frame, so the view stays
  rock-steady while the arm moves (the axes never zoom/pan per frame).

---

## 11. Implementation notes

- Math is always in **radians** internally; degree wrappers do the
  conversion.
- `acos` inputs are clamped to `[−1, 1]` to absorb floating-point error at
  the workspace boundary.
- All configurable values live in ONE configuration block at the top of
  `robot_arm.py` (`L1`, `L2`, `L3`, `WRIST_STEM_ANGLE`, `GRIPPER_FINGERS`,
  `FINGER_LENGTH`, `FINGER_OPENING_DEG`, `GRIPPER_OPENING`, `JOINT_LIMITS`,
  tolerances).
- The gripper is purely **kinematic**: the fingers are drawn and their on-axis
  grip point (centroid) is the end-effector for FK/IK/reachability, but the
  opening is a tool parameter, not a servo degree of freedom. Treating the
  stem + grip depth `(L3 + l_g)` as a fixed-angle on-axis extension keeps the
  planar solver closed-form: `L2 + (L3 + l_g)·e^(i·joint3) = m_g·e^(i·delta_g)`
  (`m_g ≈ 326.96 mm / 321.97 mm` for opening 0 / 1, `delta_g ≈ −18.53° / −17.74°`, both at
  `joint3 = −60°`). The physical elbow angle is recovered as `joint2 = te − delta_g`.
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
Base ──(L0=50 mm fixed)── joint1 ──(L1=300 mm)── joint2 ──(L2=250 mm)── wrist mount
                                                                      │  L3 (70 mm), kinked −60° off L2
                                                                      ▼
                                                                  gripper mount ── 3 fingers × 50 mm, 120° apart
```

- **Base**: the only joint attached to the world. Rotates the whole arm
  around vertical **+Z**. Range ±180°. The fixed vertical link `L0 = 50 mm`
  connects the base to `joint1` and cannot tilt.
- **`joint1`** (shoulder / L1 tilt): lifts/lowers link 1 in a vertical plane.
  Measured from horizontal outward; `joint1 = 0` → arm horizontal, `90°` → straight up.
  Range −90°..110°.
- **`joint2`** (elbow / L2 tilt): bends link 2 *relative to* link 1. `0` → arm straight.
  Range ±150°.
- **Stem `L3`**: a rigid 70 mm segment at a **fixed** `-60°` (the kink) off
  link 2. It is *not* a 4th joint — rotating `joint2` sweeps the whole stem
  around with the forearm. The stem runs from the **wrist mount** (end of L2)
  to the **gripper mount** (end of L3).
- **Fingers**: `GRIPPER_FINGERS = 3` straight sticks `FINGER_LENGTH = 50 mm`
  from the **gripper mount**, spaced 120° around the tool axis, each angled `spread` off
  it. `spread = GRIPPER_OPENING × 30°`, so `0` (closed) → fingers meet ON the
  axis, `1` (open) → they fan out to 30°.

**So the arm has exactly 4 degrees of freedom**: 4 angles (base, joint1, joint2, joint3) fully determine
every point (joint1, joint2, joint3, gripper mount, fingertips, object centre). The fingers
have no motors — their open/close is a *tool setting*, not a degree of freedom.

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

- Stem only (`l_g = 0`): `m ≈ 291.38 mm`, `delta ≈ −12.01°`.
- Grip point, open (`opening 1`): `m_g ≈ 321.97 mm`, `delta_g ≈ −17.74°`.
- Grip point, closed (`opening 0`): `m_g ≈ 326.96 mm`, `delta_g ≈ −18.53°`.

All three are evaluated at `joint3 = −60°`. The stem-only figures are independent of the
gripper; the grip-point figures vary with both `opening` **and** `joint3`.

The grip point `G = gripper_mount + l_g·axis` lies ON the tool axis
(`l_g = FINGER_LENGTH·cos(spread)`); on-axis points fold into the same planar
model — which is exactly why a **3-finger gripper adds zero mathematical
complexity**. Closing the fingers pushes `G` forward (+4.99 mm of reach) and
opening pulls it back — the source of "closing the gripper increases reach".

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
`joint1 ≈ 49.399°` and `joint2 ≈ −82.426°`, differing only in `theta_base`
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
| `JointAngles` | `theta_base`, `joint1`, `joint2` (deg) | the only output of IK |
| `FingerGripper` | `wrist`, `axis`, `spread`, `fingertips`, `grip_point`, `opening` | produced by `gripper_pose()` |
| `RobotConfig` | `l1`, `l2`, `l3`, `wrist_stem_angle`, `gripper_fingers`, `finger_length`, `finger_opening_deg`, `gripper_opening`, `JOINT_LIMITS` | one object = one arm |

**Module functions** (all take `config: RobotConfig = DEFAULT_CONFIG` and,
where noted, an optional `opening`):

| Function | Signature | Returns |
| --- | --- | --- |
| `forward_kinematics` | `(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG, opening=None)` | grip point `CartesianPoint` |
| `gripper_pose` | `(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG, opening=None)` | full `FingerGripper` geometry |
| `finger_tips` | `(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG, opening=None)` | `tuple` of 3 `CartesianPoint` |
| `inverse_kinematics` | `(x, y, z, joint2_up=True, config=DEFAULT_CONFIG, opening=None, joint3=-60.0)` | `JointAngles` (deg); `joint3` may be `"auto"` |
| `is_reachable` | `(x, y, z, config=DEFAULT_CONFIG, joint3=-60.0, opening=None)` | `bool` (geometric) |
| `check_joint_limits` | `(theta_base, joint1, joint2, joint3, config=DEFAULT_CONFIG)` | `bool` (physical) |
| `requires_joint_limits` | `(theta_base, joint1, joint2)` | `JointAngles` or raises `ValueError` |
| `verify_fk_ik` | `(n_samples=20, opening=None)` | `(max_pos_err, passed, total)` |
| `draw_robot` | `(ax, angles, target=None, opening=None, fixed_limits=None)` | draws into a 3D `Axes` |
| `plot_robot` | `(angles, target=None, opening=None)` | opens its own window |
| `interpolate_joints` | `(from_angles, to_angles, frames=60)` | eased joint-space poses (tuple) |
| `animation_limits` | `(poses, opening=None, target=None)` | one equal-aspect box enclosing the whole trajectory |
| `plot_animation` | `(from_angles, to_angles, target=None, opening=None, frames=60, interval=30)` | animates in its own window (axes stay fixed) |
| `cli` / `main` | — | the text menu |

**`RobotArm` facade** — a convenience class bound to one config. **Its kinematic methods
are currently broken** (see the facade note in §3 for which ones); the module-level
functions are the supported path:

```text
arm = ra.RobotArm()                      # default 300/250/70 arm
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
| `L1` | 300 mm | upper arm |
| `L2` | 250 mm | forearm |
| `L3` | 70 mm | wrist stem |
| `WRIST_STEM_ANGLE` | −60° | legacy default wrist angle; used only by `_grip_parameters` and the dead `_reach_bounds`. Use the `joint3` argument instead |
| `GRIPPER_FINGERS` | 3 | finger count (tripod) |
| `FINGER_LENGTH` | 50 mm | finger segment length |
| `FINGER_OPENING_DEG` | 30° | full finger spread when open |
| `GRIPPER_OPENING` | 1.0 | default opening (0 closed .. 1 open) |
| `JOINT_LIMITS` | base ±180°, j1 −90..110°, j2 ±150° | physical ranges |
| `POSITION_TOLERANCE_MM` | 0.1 | verification position bar |
| `REACH_EPSILON_MM` / `MIN_D_SHARE_LIMIT` | 1e-9 | numerical slack/guards |

Derived (reused by FK/IK/verification via `_grip_parameters`, which pins the wrist angle
to `WRIST_STEM_ANGLE = −60°` — i.e. these are the `joint3 = −60°` values):

| Quantity | Open (opening 1) | Closed (opening 0) |
| --- | --- | --- |
| grip depth `l_g` | 43.30 mm | 50.00 mm |
| effective forearm `m_g` | 321.97 mm | 326.96 mm |
| forearm offset `delta_g` | −17.74° | −18.53° |
| reach `[min, max]` at joint3 −60° | [21.97, 621.97] mm | [26.96, 626.96] mm |
| reach `[min, max]` at joint3 0° (max) | [63.30, 663.30] mm | [70.00, 670.00] mm |

Those `l_g` figures use the **full** finger length, `FINGER_LENGTH·cos(spread)`. Earlier
revisions of this table listed `21.65 / 25.0 mm` and the reach figures that follow from
them; that was a pre-tripod half-length model (`FINGER_LENGTH/2`) that the code never
used, and every downstream number in the old table inherited the error. Reach also
depends on `joint3`, so it is not one pair of numbers at all — see
[§8](#8-workspace-and-joint-limits--two-separate-ideas).

Note that `L1` (300 mm) and the effective forearm `m_g` (≈ 322 mm) are close but not
equal, giving a small but non-zero inner bound at the default `joint3`. They become
*exactly* equal at `joint3 = ±75°`, where the inner bound reaches 0.00 mm.

### 12.14 Common questions

- **Why is `joint2` the same for every target at the same distance?** The
  elbow formula depends only on `d` (§12.7).
- **Which elbow should I pick, up or down?** Whichever stays inside the joint
  limits and avoids obstacles; they both reach the same point.
- **Why does closing the gripper change the reach?** Closing moves `G`
  forward along the axis (`l_g` grows), which lengthens the effective forearm
  `m_g` and therefore the reach.
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
  current picture on the Visualization tab.

### 12.15 Glossary

- **DOF** — degree of freedom; an independently controllable joint (here: 4 - `theta_base`, `joint1`, `joint2`,
`joint3`). The gripper opening is *not* a DOF; it is a tool parameter that changes
the reach.
- **FK / IK** — forward / inverse kinematics.
- **Grip point** — on-axis centroid of the fingertips; the end-effector.
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