# IT22127464 — Dhananjaya A.G.D.
# Robot Derivation Path Mechanism Review

This project is an interactive simulation of a differential-drive robot used to review and demonstrate the “Derivation Path Mechanism” behind robot motion and control. It visualizes how P, PD, and PID controllers affect the robot’s movement toward a click-selected target while tracking performance metrics and enabling analysis. The simulation is implemented with Pygame and NumPy and includes plotting via Matplotlib.

The code emphasizes first-principles derivations for linear and angular motion and connects those equations to practical controllers and on-screen behavior.


## Key features

- Differential-drive robot model with clear math derivations from kinematics to screen motion
- Switchable controllers: P, PD, PID (with tuned default gains)
- Interactive UI (click to set destination, Start/Stop; toggle controller and performance info)
- Path visualization and basic performance monitoring (path length, energy, overshoot, settling time, etc.)
- On-demand performance analysis with charts (trajectory, error, velocities, energy)
- Extensible architecture (state machine, modular controllers, future advanced behaviors)


## How it works — core derivations

A differential-drive robot has two wheel linear velocities: $v_{left}$ and $v_{right}$. With wheel separation (wheel base) $L$:

- Linear velocity: $v = \frac{v_{left} + v_{right}}{2}$
- Angular velocity: $\omega = \frac{v_{right} - v_{left}}{L}$

Robot pose $(x, y, \theta)$ evolves by:

$$
\begin{aligned}
\frac{dx}{dt} &= v\cos\theta, \\
\frac{dy}{dt} &= v\sin\theta, \\
\frac{d\theta}{dt} &= \omega.
\end{aligned}
$$

Controllers use the distance error and heading error to produce the control signal $u$ that drives the wheel velocities:

- P: $u = K_p\,e$
- PD: $u = K_p\,e + K_d\,\frac{de}{dt}$
- PID: $u = K_p\,e + K_i\,\int e\,dt + K_d\,\frac{de}{dt}$

Here, distance error $e_d$ is the Euclidean distance to the target, and angle error $e_\theta$ is the normalized difference between target bearing and current heading. The high-level `RobotController` computes a linear command (for $v$) and an angular command (for $\omega$) from these errors, then converts $(v,\omega)$ into $(v_{left}, v_{right})$.


## Repository structure

- `robot_simulation.py` — Entry point. Initializes Pygame, sets up a finite state machine, and runs the main loop.
- `main_state.py` — The interactive state. Handles mouse/keyboard input, renders UI (buttons, labels, path, robot), converts between world and screen coordinates, and drives the robot via `RobotController`.
- `differential_drive_robot.py` — The robot model and kinematics. Maintains pose, applies wheel velocity limits/smoothing, and updates state via the derived equations.
- `controllers.py` — Modular P, PD, PID controllers and the high-level `RobotController` that blends linear and angular control, applies distance-based scaling and saturation, and outputs wheel velocities.
- `performance_monitor.py` — Tracks and computes performance metrics (arrival time, path length, energy, overshoot, settling time, steady-state error, control effort) and plots them.
- `advanced_behaviors.py` — Future extensions: smooth paths (Bezier), spiral approaches, adaptive gains, and basic formation placeholders. Not wired into the main loop by default.
- `simulation_state.py` — Simple abstract base for state machine states.
- `config.py` — Central configuration (window, colors, physical limits, tuned gains, labels, version).
- `Doucumentation/` — Assignment documents (DELIVERABLES.md, README.md, REPORT.md).


## Installation

Requirements:
- Python 3.10+ (tested with 3.12)
- Packages: pygame, numpy, matplotlib

Install packages (Windows PowerShell):

```powershell
pip install pygame numpy matplotlib
```

If you use a specific Python installation (e.g., `C:\Python\Python312\python.exe`):

```powershell
C:\Python\Python312\python.exe -m pip install pygame numpy matplotlib
```


## Running the simulation

From the project root:

```powershell
python robot_simulation.py
```

If you prefer an absolute Python path:

```powershell
& C:\Python\Python312\python.exe c:\Users\DimuthuAro\Desktop\RIS\Assesment\Dimuthu\robot_simulation.py
```

A Pygame window opens with the robot at the world origin (mapped to the screen center). Click anywhere (above the bottom command bar) to set a new destination.


## UI and controls

- Click within the main canvas to set the destination. The robot immediately starts navigating.
- Buttons (bottom bar):
  - Start — Reset robot to origin and reset controllers, clear path
  - Stop — Halt robot and reset controller internal states
  - P / PD / PID — Switch control mode
  - Monitor — Toggle performance metrics display in the top-left
  - Analyze — Print a performance report to the console and open analysis plots
  - Path — Toggle path drawing
  - Clear — Clear the accumulated path
- Keyboard:
  - T — Run an internal movement test sequence (prints to console)

Notes:
- World-to-screen mapping uses 100 px/m with the screen center as (0,0) in world coordinates. Y is flipped for screen rendering.
- Path points are throttled to avoid clutter and capped for performance.


## Configuration highlights (`config.py`)

- Timing and window:
  - `SIMULATED_SECOND = 1000`, `FPS = 60`
  - `SCREEN_WIDTH = 800`, `SCREEN_HEIGHT = 600`, `TITLE = "RIS Assignment -- Differential Drive Robot Simulation 1.0.0"`
- Colors: predefined RGB tuples for UI and drawing
- Robot and control:
  - `WHEEL_BASE = 0.3` m, `MAX_LINEAR_VELOCITY = 3.0` m/s, `MAX_ANGULAR_VELOCITY = 5.0` rad/s
  - `MAX_ACCELERATION = 5.0` m/s² (used for velocity smoothing)
  - Tolerances: `POSITION_TOLERANCE = 0.03` m, `ANGLE_TOLERANCE = 0.1` rad
- Tuned gains (defaults used by `RobotController`):
  - P: linear Kp=1.5, angular Kp=4.0
  - PD: linear Kp=2.5, Kd=0.8; angular Kp=4.5, Kd=1.2
  - PID: linear Kp=3.0, Ki=0.05, Kd=0.7; angular Kp=5.0, Ki=0.02, Kd=1.0

You can tweak these values to study the effect on motion, overshoot, and settling.


## Performance monitoring and analysis

While running, toggle “Monitor” to see basic metrics inline. Press “Analyze” to:
- Print a summary (arrival time, path length, energy, overshoot, settling, steady-state error, control effort)
- Show Matplotlib plots for trajectory, error, velocities, control signals, energy, and a metrics panel
- Receive optimization suggestions (e.g., adjust Kp/Kd/Ki) based on observed behavior

Tip: Switch between P/PD/PID and compare plots to understand how derivative and integral actions shape the response.


## Advanced behaviors (preview)

The `advanced_behaviors.py` module includes:
- Curved path generation via cubic Bezier interpolation
- Spiral approach waypoints for precise docking
- An adaptive controller scaffold that adjusts gains based on observed performance
- Simple formation logic (leader-follower offsets)

These are not integrated into `main_state.py` by default but are suitable for experiments and future extensions. A typical integration pattern is to produce waypoints with `RobotBehavior` and command the robot to track them sequentially using the existing `RobotController`.


## Tuning guidance

- Start with P control and increase `Kp` until response is fast but not unstable
- Add `Kd` (PD) to reduce overshoot and oscillations
- Add `Ki` (PID) to remove steady-state error; use integral limits to prevent windup
- Use “Analyze” to corroborate tuning with metrics and plots


## Known limitations

- No obstacle modeling; path planning is straight-line to target (advanced path generators are provided but not wired in)
- Single robot in the main application (formation code is a placeholder for future work)
- Physics is purely kinematic; no slippage or dynamics


## Credits

- Student: IT22127464 — Dhananjaya A.G.D.
- Course context: RIS Assignment — Differential Drive Robot Simulation
- Docs: see `Doucumentation/` for assignment deliverables and report


## Troubleshooting

- If the window does not open or crashes, ensure Pygame is installed and that your GPU/driver supports basic 2D rendering.
- If plots do not show, verify Matplotlib is installed and your Python environment is the one used to run the app.
- On high-DPI displays, adjust `meters_to_pixels` in `main_state.py` for preferred scaling.
