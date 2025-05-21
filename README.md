# 🛸 UAV Interception Simulator

This project simulates a real-time 3D drone interception scenario where an autonomous interceptor drone tracks and intercepts a flying UFO using smooth trajectory planning and cascaded PID control.

## 🚀 Project Overview

### Components:
- **UFO (Target Drone)**
  A simulated target with 3D motion defined by initial position, velocity, and constant acceleration.

- **Stryxceptor (Interceptor Drone)**
  An autonomous quadrotor-style drone that follows an optimal trajectory to intercept the UFO from behind. 

- **Motion Planner**
  Predicts the UFO's future trajectory and computes a smooth polynomial interception path based on target dynamics and constraints.

- **Simulation Loop**
  Handles time-stepping, physical updates, and visualization using matplotlib.

- **Rendering**
  3D animation of the interception scenario or static trajectory rendering.

## 📁 Repository Structure

```bash
.
├── Simulate.py         # Main entry point: runs the simulation and animation / Have arguments to play with
├── UFO.py              # Very simple class that defines a UFO (target)  
├── Motion_Planner.py   # Trajectory prediction and optimal path computation
├── Stryxceptor.py      # (Expected) Interceptor class with control logic
└── README.md           # You are here

## ⚙️ Requirements

- Python 3.8+
- NumPy
- Matplotlib

Install dependencies:

```bash
pip install numpy matplotlib

## ▶️ How to use 

Simply run the simulation with:
```bash
python Simulate.py

You should see overall information on the terminal and a plot of the interception.

To run an animation instead, set live_rendering = True inside Simulate.py.

## 🧠 Underlying Hypothesis

- Interceptor is modelled by a point with acceleration, speed and rotation constraints 
- UFO is modelled as a point with constant acceleration (no world dynamics applied)
- No obstacle is considered for the trajectory (although it would be quite easy to tackle with an A* algo for the waypoints selection)

# 📌 Still To Do 

- Implement a Control logic for the Interceptor (either model-based of PID) taking into account the constraints 
  --> Now just following the trajectory without hard-coded feasibility check  
- Complexify the UFO behavior (Feed a random trajectory instead)
- Implement recomputation of the trajectory live (not much work but meaningless for current UFO) 
- Complexify Interceptor model
- Trajectory computation can be further improved (choice of waypoints, directly feasible trajectories)
  --> Inspiration could be drawn from : https://ieeexplore.ieee.org/document/8206119 // https://ieeexplore.ieee.org/document/9422918





