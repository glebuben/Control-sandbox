# CONTROL-SANDBOX

This git is a collection of advanced control methods implemented on Python. Each project uses on a different nonlinear control strategy applied to a benchmark dynamical system.

## Repository structure
CONTROL-SANDBOX/    
├── Project_1_Lyapunov_based_control_cartpole/  
├── Project_2_Adaptive_control_name/    
├── Project_3_Backstepping_name/    
├── Project_4_Model_predictive_control_name/    
├── .gitignore  
├── README.md   
└── requirements.txt

## Projects overview

### Project 1 – Lyapunov‑based control of CartPole (completed)

**System:** CartPole (cart + inverted pendulum)  
**Control method:** Sliding Mode Control (SMC) with Lyapunov function   
**Goal:** Stabilise the pendulum in the upright unstable position and bring the cart to the origin.
![alt text](Project_1_Lyapunov_based_control_cartpole/animations/cartpole_lqr.gif)
**Run Project 1:**
```bash
cd Project_1_Lyapunov_based_control_cartpole/src
python main.py         
```
*Results:* The pendulum is stabilised, the cart returns to zero, and the Lyapunov function decays monotonically.

### Project 2 – Adaptive control (under maintenance)
Planned implementation of an adaptive controller 

### Project 3 – Backstepping control (under maintenance)
Will demonstrate the backstepping design methodology.  A control function will be constructed step by step.

### Project 4 – Model Predictive Control (under maintenance)
Will implement a nonlinear MPC for a constrained system 

### Installation and dependencies
All projects require Python 3.8+ and the following packages:

numpy  
scipy   
matplotlib

#### Install dependencies from the root directory:

```
pip install -r requirements.txt
```

#### Contents of requirements.txt:

```
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.4.0
```

### General notes

Each project has its own folder with a consistent layout:

configs/ – system and controller parameters

src/ – source code (system.py, controller.py, simulation.py, visualization.py, main.py)

animations/, figures/ – generated output files

README.md – detailed description of the project (inside its folder)

### License
Educational use only. Free for non‑commercial purposes.

### Authors (under maintenace for responsibility of each project)
Team Lazy controller
