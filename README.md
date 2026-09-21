\# Non-linear Dynamics \& Control of an Inverted Pendulum



Mathematical modeling and closed-loop control system development for an inverted pendulum on a cart, implemented in MATLAB and Simulink. 



\## Key Features \& Methodology



1\. \*\*Mathematical Modeling:\*\*

&#x20;  \* Derivation of the non-linear equations of motion using Lagrangian mechanics.

&#x20;  \* Linearization of the system around the unstable vertical equilibrium point.

&#x20;  \* Extraction of the linearized state-space model (A, B, C, D matrices).

2\. \*\*Control System Design:\*\*

&#x20;  \* \*\*PID Control:\*\* Design and tuning of a Proportional-Integral-Derivative controller for baseline stabilization.

&#x20;  \* \*\*State Feedback Control:\*\* Implementation of an optimal control strategy (e.g., LQR / Pole Placement) utilizing the full state-space dynamics for improved robustness and response time.

3\. \*\*Simulation:\*\*

&#x20;  \* Full non-linear block diagram simulation in Simulink to validate the control laws against realistic physical constraints.



\## Repository Structure



```text

\- docs/pendolo\_inverso       # Project report and mathematical derivations

\- models/                    # Simulink block diagrams

&#x20; ├── Model\_40\_01\_1.slx      # Open-loop non-linear plant simulation

&#x20; ├── Model\_40\_01\_2.slx      # Closed-loop simulation with PID controller

&#x20; └── Model\_40\_01\_3.slx      # Closed-loop simulation with State Feedback control

\- main\_pendulum.m            # Master script for parameters and state-space matrices

\- README.md                  # This file

```

