# Non-linear Dynamics & Control of an Inverted Pendulum

Mathematical modeling and closed-loop control system development for an inverted pendulum on a cart, implemented in MATLAB and Simulink.

## Project Overview

The objective of this project is to stabilize an inherently unstable non-linear system (the inverted pendulum) in its upright position while controlling the cart's lateral position.

## Key Features & Methodology

**1. Mathematical Modeling:**
* Derivation of the non-linear equations of motion using Lagrangian mechanics.
* Linearization of the system around the unstable vertical equilibrium point.
* Extraction of the linearized state-space model (A, B, C, D matrices).

**2. Control System Design:**
* **PID Control:** Design and tuning of a Proportional-Integral-Derivative controller for baseline stabilization.
* **State Feedback Control:** Implementation of an optimal control strategy utilizing the full state-space dynamics for improved robustness and response time.

**3. Simulation:**
* Full non-linear block diagram simulation in Simulink to validate the control laws against realistic physical constraints.

## Repository Structure

- `docs/pendolo_inverso` Project report and mathematical derivations
- `models/` Simulink block diagrams
  - `Model_40_01_1.slx` Open-loop non-linear plant simulation 
  - `Model_40_01_2.slx` Closed-loop simulation with PID controller 
  - `Model_40_01_3.slx` State-space control with Observer on non-linear plant 
- `Code_40_01.m` Master script 
- `README.md` This file
