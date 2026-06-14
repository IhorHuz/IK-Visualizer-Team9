# IK-Visualizer-Team9

Hello to one and all. This repository is created to organise our work as a team during the Lodz University of Technology Team Project course.

This repository contains the source code, research, and documentation for the project. Primarily, it is an educational interactive 3D sandbox for visualizing Inverse Kinematics (IK) algorithms on a 6-DOF robot arm in real time. Built with Godot 4 and Python.

## How to Run

1. **Start the Python IK server**:
   In the root folder of the project, run:

   ```bash
   python3 math_engine/ik_server3d.py
   ```

   You should see: `Python 3D IK Server running on 127.0.0.1:5005...`

2. **Open the Godot project** at `godot_project/ik-visualizer/` and run the scene `Main3D.tscn`.

3. The arm will try to reach the yellow sphere. Move it to see the solvers in action.

## Controls

| Key                    | Action                                                       |
| ---------------------- | ------------------------------------------------------------ |
| **Left click + drag**  | Move target sphere on the ground plane                       |
| **Right click + drag** | Orbit camera around the scene                                |
| **Up / Down arrows**   | Move target sphere vertically (Y-axis)                       |
| **V**                  | Toggle between Stick Arm (skeleton) and Robot Arm (3D model) |
| **1–6**                | Select joint J0–J5 (enters manual-override mode)             |
| **Q / W (hold)**       | Decrease / increase selected joint angle (manual mode)       |
| **R**                  | Exit manual-override mode, restore solver-driven IK          |
| **L**                  | Toggle lock on selected joint (frozen by IK solvers)         |
| **T**                  | Toggle endpoint motion trail (shows convergence path)        |
| **Dropdown**           | Switch between CCD / FABRIK / Jacobian algorithms            |

## Robot Arm Configuration

### Joints (6 DOF)

| Joint | Axis | Limits    | Segment Length | Description    |
| ----- | ---- | --------- | -------------- | -------------- |
| J0    | Y    | unbounded | 1.2            | Base yaw       |
| J1    | Z    | ±90°      | 3.1            | Shoulder pitch |
| J2    | Z    | ±90°      | 0.85           | Elbow pitch    |
| J3    | Y    | unbounded | 1.3            | Wrist yaw      |
| J4    | Z    | ±90°      | 5.25           | Wrist pitch    |
| J5    | Z    | ±90°      | 0.825          | Endpoint pitch |

The arm's max reach is shown when the python math server is started.

### Joint Lock

Freeze individual joints while the arm follows the target via IK:

1. Press **1–6** to select a joint
2. Press **R** to exit manual-override mode (keeps joint selected)
3. Press **L** to toggle lock — the solver skips that joint
4. Drag the sphere — locked joint stays frozen, rest of arm reaches

Press **L** again to unlock. Press **V** twice to clear all locks.

## IK Algorithms

| Algorithm    | Type      | Description                                         |
| ------------ | --------- | --------------------------------------------------- |
| **CCD**      | Iterative | Joint-by-joint from tip to base — simple and robust |
| **FABRIK**   | Iterative | Two-pass position-based solver — fast convergence   |
| **Jacobian** | Iterative | Matrix-based gradient descent with DLS damping      |

Select from the dropdown in the UI. Only one solver runs per frame; switch to compare behavior.

## Architecture

```
┌─────────────┐     UDP (port 5005)     ┌──────────────────┐
│   Godot 4   │ ◄─────────────────────► │  Python Server   │
│  (visual)   │    target & angles      │  (ik_server3d)   │
└─────────────┘                         └────────┬─────────┘
                                                 │
                                    ┌────────────┼────────────┐
                                    │            │            │
                              ┌─────┴──┐  ┌──────┴─────┐  ┌──┴─────┐
                              │  CCD   │  │  FABRIK    │  │Jacobian│
                              └────────┘  └────────────┘  └────────┘
```

One frame per UDP message: Godot sends target position → server runs one solver iteration → returns joint angles → Godot renders.

## File Structure

```
math_engine/
├── ik_server3d.py           # UDP server, dispatches to solvers
├── robot/                   # DOF-constrained solvers
│   ├── ccd_robot_3d.py      # CCD solver
│   ├── fabrik_robot_3d.py   # FABRIK solver
│   └── jacobian_robot_3d.py # Jacobian DLS solver
└── stick/                   # Unconstrained solvers for the stick arm mode
    ├── ccd_3d.py
    ├── fabrik_3d.py
    └── jacobian_3d.py

godot_project/ik-visualizer/
├── spatial_bridge.gd        # Main script (rendering, controls, UDP)
├── Main3D.tscn              # Main scene
├── visuals/                 # FBX robot arm models
└── ...
```

## Troubleshooting

| Problem                  | Fix                                                  |
| ------------------------ | ---------------------------------------------------- |
| "Address already in use" | `pkill -f ik_server` then restart                    |
| Arm not moving           | Check server is running (`ps aux \| grep ik_server`) |
| Unreachable target       | Target is outside the arm's ±90° pitch workspace     |
