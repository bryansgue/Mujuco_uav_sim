# A Modular High-Fidelity Quadrotor Simulator Based on MuJoCo and ROS 2: From Simplified Wrench Models to Realistic Motor Dynamics for Digital Twin Applications

---

**Authors:** Bryan S. Guevara et al.

**Abstract —** This paper presents a modular, open-source quadrotor simulation framework built upon the MuJoCo physics engine and the ROS 2 middleware. The simulator implements two distinct actuation models with increasing physical fidelity: (i) a *Direct Wrench Model* that applies aggregate thrust and torque to the rigid body, and (ii) a *Motor-Level Model* that incorporates individual propeller aerodynamics, a control allocation mixer in Betaflight X-configuration, first-order motor dynamics, and body-frame aerodynamic drag. Both models share a common inner-loop angular rate controller based on gyroscopic compensation and proportional feedback. The mathematical formulation of every subsystem is presented in detail, including the rigid-body dynamics, the quaternion-based attitude representation, the allocation matrix derivation, the motor electromechanical response, and the propeller thrust–torque relationships. The framework is designed as a digital twin platform, where each parameter can be calibrated against a physical Betaflight-based UAV. A ROS 2 plugin architecture ensures modularity, reproducibility, and seamless integration with external controllers such as Nonlinear Model Predictive Control (NMPC). Numerical considerations regarding integration schemes, time-stepping, and simulation stability are discussed. The simulator is validated qualitatively through hover and trajectory tracking experiments in both actuation modes.

**Keywords:** Quadrotor simulation, MuJoCo, ROS 2, motor dynamics, digital twin, Betaflight, control allocation, UAV.

---

## I. Introduction

Unmanned Aerial Vehicles (UAVs), particularly quadrotors, have become ubiquitous platforms for research in autonomous navigation, aerial manipulation, and payload transportation. The rapid development of control algorithms — from classical PID cascades to advanced Nonlinear Model Predictive Control (NMPC) and reinforcement learning — demands simulation environments that are simultaneously *physically accurate*, *computationally efficient*, and *easily interfaced* with real-time control software.

The current landscape of quadrotor simulation can be broadly categorized into three tiers of physical fidelity:

1. **Kinematic/simplified dynamics simulators** (e.g., MATLAB scripts, simple Python integrators) that model the quadrotor as a point mass or rigid body with ideal actuation. These are fast but neglect critical phenomena such as motor lag, propeller saturation, and aerodynamic drag.

2. **Mid-fidelity physics engines** (e.g., Gazebo with ODE/Bullet, AirSim with PhysX) that provide rigid-body contact dynamics and basic aerodynamic plugins. However, the integration accuracy and actuator modeling fidelity are often limited by the underlying solver.

3. **High-fidelity computational fluid dynamics (CFD)** coupled simulations that capture blade-element aerodynamics and rotor–rotor interaction. These are prohibitively expensive for real-time or near-real-time control development.

MuJoCo (Multi-Joint dynamics with Contact) [1] occupies a unique position in this landscape. Originally developed for biomechanical simulation and robotic locomotion, MuJoCo employs a convex optimization-based contact solver and supports high-order implicit and explicit integrators (Euler, RK4, implicit) with configurable timesteps. Its C API, deterministic simulation, and plugin architecture make it an excellent foundation for control-oriented UAV simulation.

This paper presents a modular quadrotor simulation framework that leverages MuJoCo's physics engine through a set of custom C++ plugins integrated with ROS 2 Humble. The key contributions are:

- A rigorous mathematical description of two actuation models — *Direct Wrench* and *Motor-Level* — both operating within the same simulation infrastructure.
- Derivation of the control allocation (mixer) matrix for the Betaflight X-configuration, including the inverse mapping from desired wrench to individual motor speed commands.
- Integration of first-order motor electromechanical dynamics and propeller aerodynamic models into the MuJoCo simulation loop.
- A modular ROS 2 plugin architecture that enables seamless switching between actuation fidelity levels and straightforward integration with external controllers.
- Discussion of numerical considerations, parameter calibration strategies, and the path toward a fully calibrated digital twin.

The remainder of this paper is organized as follows. Section II reviews related work. Section III establishes the mathematical preliminaries and coordinate frame conventions. Section IV derives the complete rigid-body dynamics of the quadrotor. Section V presents the Direct Wrench actuation model and the inner-loop rate controller. Section VI derives the Motor-Level model including the mixer, motor dynamics, and propeller aerodynamics. Section VII discusses the MuJoCo simulation engine configuration and numerical integration. Section VIII describes the software architecture and ROS 2 integration. Section IX presents simulation results. Section X discusses calibration strategies for digital twin applications. Section XI concludes the paper.

---

## II. Related Work

### A. Quadrotor Dynamic Modeling

The rigid-body dynamics of a quadrotor have been extensively studied in the literature. Mahony et al. [2] established the SE(3) formulation widely used in geometric control. Mellinger and Kumar [3] developed trajectory optimization frameworks based on differential flatness, assuming ideal actuation. Faessler et al. [4] highlighted the importance of including actuator dynamics in aggressive flight control. More recently, Torrente et al. [5] demonstrated that accounting for aerodynamic effects and motor lag significantly improves the sim-to-real transfer of learned controllers.

### B. Simulation Platforms

Gazebo [6], coupled with RotorS [7], has been the de facto standard for ROS-based UAV simulation, providing aerodynamic plugins and motor models within a Bullet/ODE physics backend. However, the integration accuracy and solver stability of these backends are limited at the small timesteps required for fast inner-loop controllers.

AirSim [8] (now Colossus) employs Unreal Engine for rendering and PhysX for dynamics, targeting primarily vision-based autonomy. Its rigid-body dynamics are less configurable than MuJoCo's.

Flightmare [9] provides a lightweight rendering and dynamics engine specifically for quadrotors, but uses a simplified Euler integrator and lacks the general-purpose contact dynamics that MuJoCo provides.

The recent open-sourcing of MuJoCo [1] has led to its adoption in legged robotics and manipulation. Its use in aerial robotics remains relatively unexplored, which this work aims to address.

### C. Digital Twin Concepts for UAVs

The digital twin paradigm [10] requires a simulation model whose parameters are identified from the physical system. For quadrotors, this means calibrating inertial properties, motor time constants, propeller coefficients, and drag parameters from bench tests and flight data. Our framework is designed with this calibration pipeline in mind, exposing all relevant parameters through the simulation configuration.

---

## III. Mathematical Preliminaries

### A. Reference Frames

We define two principal reference frames:

- **World frame** $\mathcal{W} = \{O_W, \mathbf{e}_1^W, \mathbf{e}_2^W, \mathbf{e}_3^W\}$: An inertial frame with $\mathbf{e}_3^W$ pointing upward (against gravity).
- **Body frame** $\mathcal{B} = \{O_B, \mathbf{e}_1^B, \mathbf{e}_2^B, \mathbf{e}_3^B\}$: Fixed to the quadrotor's center of mass, with $\mathbf{e}_1^B$ pointing forward, $\mathbf{e}_2^B$ pointing left, and $\mathbf{e}_3^B$ pointing upward through the propeller plane.

### B. Rotation Representation

The orientation of $\mathcal{B}$ with respect to $\mathcal{W}$ is represented by the rotation matrix $\mathbf{R} \in SO(3)$, which maps vectors from body to world coordinates:

$$\mathbf{v}^W = \mathbf{R} \, \mathbf{v}^B$$

Internally, MuJoCo stores orientation as a unit quaternion $\mathbf{q} = (q_w, q_x, q_y, q_z) \in \mathbb{S}^3$ with the Hamilton convention ($q_w$ is the scalar part). The rotation matrix is reconstructed as:

$$\mathbf{R}(\mathbf{q}) = \begin{bmatrix}
1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_w q_z) & 2(q_x q_z + q_w q_y) \\
2(q_x q_y + q_w q_z) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_w q_x) \\
2(q_x q_z - q_w q_y) & 2(q_y q_z + q_w q_x) & 1 - 2(q_x^2 + q_y^2)
\end{bmatrix}$$

This avoids the gimbal lock singularity inherent to Euler angle representations and provides numerically stable integration of the rotational kinematics.

### C. Angular Velocity

Let $\boldsymbol{\omega}^W \in \mathbb{R}^3$ denote the angular velocity of $\mathcal{B}$ expressed in $\mathcal{W}$, as returned by MuJoCo's `mj_objectVelocity()` function. The body-frame angular velocity is obtained via:

$$\boldsymbol{\omega}^B = \mathbf{R}^\top \boldsymbol{\omega}^W$$

The components are $\boldsymbol{\omega}^B = (\omega_x, \omega_y, \omega_z)^\top$, corresponding to roll rate ($p$), pitch rate ($q$), and yaw rate ($r$) respectively.

### D. Notation Summary

| Symbol | Description | Units |
|--------|-------------|-------|
| $m$ | Total quadrotor mass | kg |
| $\mathbf{J}$ | Inertia tensor (body frame) | kg·m² |
| $\mathbf{p}^W$ | Position in world frame | m |
| $\mathbf{v}^W$ | Linear velocity in world frame | m/s |
| $\mathbf{R}$ | Rotation matrix ($\mathcal{B} \to \mathcal{W}$) | — |
| $\boldsymbol{\omega}^B$ | Angular velocity in body frame | rad/s |
| $F$ | Total thrust along $\mathbf{e}_3^B$ | N |
| $\boldsymbol{\tau}$ | Torque vector in body frame | N·m |
| $g$ | Gravitational acceleration ($9.81$) | m/s² |

---

## IV. Quadrotor Rigid-Body Dynamics

### A. Translational Dynamics

The quadrotor is modeled as a single rigid body of mass $m$ subject to gravity, the total thrust force $F$ along the body $z$-axis, and an optional aerodynamic drag force. The Newton–Euler equations in the world frame are:

$$m \ddot{\mathbf{p}}^W = -m g \, \mathbf{e}_3^W + \mathbf{R} \begin{pmatrix} 0 \\ 0 \\ F \end{pmatrix} + \mathbf{f}_{\text{ext}}$$

where $\mathbf{f}_{\text{ext}}$ includes contact forces (computed internally by MuJoCo's constraint solver) and any external disturbances.

In MuJoCo, translational dynamics are handled by the engine's generalized coordinate solver. The thrust force $F$ is applied through a MuJoCo *motor actuator* attached to a site coincident with the body's center of mass, with a gear vector $\mathbf{g}_F = (0, 0, 1, 0, 0, 0)$ that maps the scalar control input to a force along the body $z$-axis.

### B. Rotational Dynamics

The Euler equation for rotational dynamics in the body frame is:

$$\mathbf{J} \dot{\boldsymbol{\omega}}^B = \boldsymbol{\tau}^B - \boldsymbol{\omega}^B \times (\mathbf{J} \boldsymbol{\omega}^B)$$

where $\boldsymbol{\tau}^B = (\tau_x, \tau_y, \tau_z)^\top$ is the net external torque in the body frame, and the term $\boldsymbol{\omega}^B \times (\mathbf{J} \boldsymbol{\omega}^B)$ is the gyroscopic (Coriolis) coupling.

The inertia tensor is assumed diagonal due to the quadrotor's symmetric geometry:

$$\mathbf{J} = \text{diag}(J_{xx}, J_{yy}, J_{zz}) = \text{diag}(3.454 \times 10^{-3}, \; 1.797 \times 10^{-3}, \; 1.797 \times 10^{-3}) \; \text{kg·m}^2$$

These values are computed by MuJoCo from the composite geometry of the quadrotor model, which consists of:

| Component | Geometry | Mass (kg) | Quantity |
|-----------|----------|-----------|----------|
| Central body | Box: $70 \times 70 \times 30$ mm | $0.850$ | 1 |
| Arms | Box: $100 \times 20 \times 5$ mm | $0.035$ | 4 |
| Propeller discs | Cylinder: $r = 30$ mm, $h = 5$ mm | $0.015$ | 4 |

The total mass is:

$$m = 0.850 + 4 \times 0.035 + 4 \times 0.015 = 1.05 \; \text{kg}$$

> **Note:** MuJoCo computes the composite inertia tensor automatically from the constituent geometries using the parallel axis theorem. The total mass reported by MuJoCo is $m = 1.05$ kg (the value $1.08$ kg used in the controller accounts for additional unmodeled mass).

### C. Quadrotor Geometry

The quadrotor has an X-configuration with arm length $L = 0.1$ m measured from the center of mass to each motor mount. The arms are oriented at $\pm 45°$ relative to the body $x$-axis, placing the motors at body-frame coordinates:

| Motor | Body Position | Spin Direction |
|-------|--------------|----------------|
| M1 (front-right) | $(+L_s, -L_s, 0)$ | CW |
| M2 (rear-right) | $(-L_s, -L_s, 0)$ | CCW |
| M3 (rear-left) | $(-L_s, +L_s, 0)$ | CW |
| M4 (front-left) | $(+L_s, +L_s, 0)$ | CCW |

where $L_s = L / \sqrt{2}$ is the effective moment arm projected onto the body $x$ and $y$ axes for the X-configuration. This follows the Betaflight motor numbering convention.

---

## V. Model I: Direct Wrench Actuation

### A. Overview

The Direct Wrench Model (referred to as the "simplified model" hereafter) bypasses individual motor and propeller modeling entirely. The control architecture consists of:

1. An **external controller** (e.g., PD, NMPC) that computes a desired thrust $F_{\text{cmd}}$ and desired body-frame angular velocity $\boldsymbol{\omega}_d = (\omega_{d,x}, \omega_{d,y}, \omega_{d,z})^\top$.
2. An **inner-loop rate controller** (the AcroMode plugin) that converts the angular velocity command into body-frame torques.
3. **Direct application** of the resulting wrench $(F, \tau_x, \tau_y, \tau_z)$ to MuJoCo actuators.

### B. Inner-Loop Rate Controller

The rate controller implements a proportional feedback law with feedforward gyroscopic compensation. Given the current body-frame angular velocity $\boldsymbol{\omega}^B$ (obtained from MuJoCo) and the desired angular velocity $\boldsymbol{\omega}_d$ (received via ROS 2), the control torque is:

$$\boldsymbol{\tau}_{\text{des}} = \boldsymbol{\omega}^B \times (\mathbf{J} \boldsymbol{\omega}^B) - \mathbf{J} \mathbf{K}_{\omega} (\boldsymbol{\omega}^B - \boldsymbol{\omega}_d)$$

where $\mathbf{K}_{\omega} = \text{diag}(K_{\omega,x}, K_{\omega,y}, K_{\omega,z})$ is the diagonal gain matrix.

**Physical interpretation:**

- The term $\boldsymbol{\omega}^B \times (\mathbf{J} \boldsymbol{\omega}^B)$ is a **gyroscopic feedforward** that cancels the Coriolis coupling in the Euler equation. Substituting this control law into the rotational dynamics yields:

$$\mathbf{J} \dot{\boldsymbol{\omega}}^B = -\mathbf{J} \mathbf{K}_{\omega} (\boldsymbol{\omega}^B - \boldsymbol{\omega}_d)$$

- This results in a decoupled, first-order convergence on each axis:

$$\dot{\omega}_i = -K_{\omega,i} (\omega_i - \omega_{d,i}), \quad i \in \{x, y, z\}$$

with time constants $\tau_i = 1 / K_{\omega,i}$.

**Controller parameters:**

| Axis | Gain $K_{\omega,i}$ | Time constant $\tau_i$ (ms) |
|------|---------------------|------------------------------|
| Roll ($x$) | $20.0$ | $50.0$ |
| Pitch ($y$) | $35.0$ | $28.6$ |
| Yaw ($z$) | $45.0$ | $22.2$ |

The asymmetric gains reflect the fact that the roll axis has a larger moment of inertia ($J_{xx} > J_{yy} = J_{zz}$) and thus requires less aggressive tracking, while the yaw axis is tuned for faster response.

### C. Actuator Mapping

In the Direct Wrench Model, the computed wrench is written directly to four MuJoCo motor actuators:

| Actuator | Control Signal | Gear Vector | Range |
|----------|---------------|-------------|-------|
| `body_thrust` | $F_{\text{cmd}}$ | $(0,0,1,0,0,0)$ | $[0, 82]$ N |
| `x_moment` | $\tau_{x}$ | $(0,0,0,1,0,0)$ | $[-0.3, 0.3]$ N·m |
| `y_moment` | $\tau_{y}$ | $(0,0,0,0,1,0)$ | $[-0.3, 0.3]$ N·m |
| `z_moment` | $\tau_{z}$ | $(0,0,0,0,0,1)$ | $[-0.3, 0.3]$ N·m |

The gear vector specification $(g_1, g_2, g_3, g_4, g_5, g_6)$ in MuJoCo maps a scalar control input $u$ to a generalized force: the first three components define a force vector and the last three define a torque vector, all in the site's local frame.

### D. Control Execution Rate

The rate controller executes at a configurable frequency $f_c = 500$ Hz. Given MuJoCo's simulation timestep of $\Delta t = 3$ ms (corresponding to approximately $333$ Hz), the controller updates at roughly every $1.5$ simulation steps. This is managed via an internal timer that accumulates simulation time and triggers control updates at the specified rate:

$$t_{\text{next}} = t_{\text{next}} + \frac{1}{f_c}$$

A `while` loop ensures that multiple control updates are executed if the simulation advances by more than one control period, preventing control lag during large timestep configurations.

### E. Advantages and Limitations

**Advantages:**
- Computationally trivial: no mixer computation, no motor state integration.
- Direct mapping of control intent to body-level forces/torques.
- Ideal for control algorithm development where actuator dynamics are not the focus.
- Guaranteed bounded actuator outputs through MuJoCo's `ctrllimited` constraint.

**Limitations:**
- No representation of actuator bandwidth: wrench changes are instantaneous.
- No coupling between thrust and torque through shared actuators (motors).
- Cannot capture motor saturation, propeller stall, or the non-trivial mapping from desired wrench to achievable wrench.
- Poor sim-to-real transfer for aggressive maneuvers where motor dynamics are significant.

---

## VI. Model II: Motor-Level Actuation (Digital Twin)

### A. Overview

The Motor-Level Model introduces a realistic actuation pipeline between the rate controller output and the MuJoCo actuators. This pipeline models:

1. **Control allocation** (mixer): mapping desired wrench to individual motor speed commands.
2. **Motor electromechanical dynamics**: first-order response of each motor.
3. **Propeller aerodynamics**: thrust and reactive torque as functions of motor speed.
4. **Aerodynamic drag**: body-frame linear drag opposing translational motion.

The resulting architecture is:

$$\underbrace{(F_{\text{cmd}}, \boldsymbol{\omega}_d)}_{\text{External command}} \xrightarrow{\text{Rate controller}} \underbrace{(F_{\text{des}}, \boldsymbol{\tau}_{\text{des}})}_{\text{Desired wrench}} \xrightarrow{\text{Mixer}} \underbrace{(\Omega_{1,d}^2, \ldots, \Omega_{4,d}^2)}_{\text{Desired motor speeds}} \xrightarrow{\text{Motor dynamics}} \underbrace{(\Omega_1, \ldots, \Omega_4)}_{\text{Actual motor speeds}} \xrightarrow{\text{Propeller model}} \underbrace{(F_{\text{act}}, \boldsymbol{\tau}_{\text{act}})}_{\text{Actual wrench}}$$

### B. Propeller Aerodynamic Model

Each propeller $i$ generates a thrust force $f_i$ and a reactive torque $\mu_i$ that are functions of the motor angular velocity $\Omega_i$:

$$f_i = k_f \, \Omega_i^2$$

$$\mu_i = k_m \, \Omega_i^2$$

where:
- $k_f$ [N/(rad/s)²] is the **thrust coefficient**, determined by blade geometry, airfoil, number of blades, air density, and propeller radius.
- $k_m$ [N·m/(rad/s)²] is the **torque coefficient** (also called drag coefficient of the propeller), related to the aerodynamic drag on the blades.

The ratio $k_m / k_f$ has units of meters and is related to the blade's drag-to-lift ratio:

$$\frac{k_m}{k_f} = c_{\tau f} \approx 0.0136 \; \text{m}$$

This is a characteristic constant of the propeller that is relatively insensitive to operating conditions and can be measured on a thrust stand.

**Blade Element Theory justification:** The quadratic relationship $f \propto \Omega^2$ follows from dimensional analysis and blade element momentum theory (BEMT). For a propeller of radius $R$ operating in hover (zero advance ratio), the thrust is:

$$f = C_T \rho n^2 D^4$$

where $C_T$ is the thrust coefficient, $\rho$ the air density, $n = \Omega/(2\pi)$ the rotational frequency, and $D = 2R$ the diameter. Since $C_T$ is approximately constant at fixed pitch, $f \propto \Omega^2$.

**Default parameter values:**

| Parameter | Symbol | Value | Derivation |
|-----------|--------|-------|------------|
| Thrust coefficient | $k_f$ | $1.91 \times 10^{-6}$ N/(rad/s)² | Hover condition: $f_{\text{hover}} = mg/4 = 2.65$ N at $\Omega_{\text{hover}} \approx 1178$ rad/s |
| Torque coefficient | $k_m$ | $2.6 \times 10^{-8}$ N·m/(rad/s)² | Typical ratio $k_m/k_f \approx 0.0136$ |

### C. Control Allocation Matrix (Mixer)

The total wrench on the quadrotor body due to the four propellers is the sum of individual contributions. For the Betaflight X-configuration with the motor layout defined in Section IV-C:

**Thrust (body $z$-axis):** All four propellers contribute positively:

$$F = k_f (\Omega_1^2 + \Omega_2^2 + \Omega_3^2 + \Omega_4^2)$$

**Roll torque ($\tau_x$):** Generated by differential thrust between left-side (M3, M4) and right-side (M1, M2) motors, with effective moment arm $L_s = L/\sqrt{2}$:

$$\tau_x = k_f L_s (-\Omega_1^2 - \Omega_2^2 + \Omega_3^2 + \Omega_4^2)$$

**Pitch torque ($\tau_y$):** Generated by differential thrust between front (M1, M4) and rear (M2, M3) motors:

$$\tau_y = k_f L_s (+\Omega_1^2 - \Omega_2^2 - \Omega_3^2 + \Omega_4^2)$$

**Yaw torque ($\tau_z$):** Generated by the reactive torques of the propellers. CW-spinning motors (M1, M3) produce negative yaw torque, while CCW-spinning motors (M2, M4) produce positive yaw torque:

$$\tau_z = k_m (-\Omega_1^2 + \Omega_2^2 - \Omega_3^2 + \Omega_4^2)$$

In compact matrix form:

$$\underbrace{\begin{pmatrix} F \\ \tau_x \\ \tau_y \\ \tau_z \end{pmatrix}}_{\mathbf{w}} = \underbrace{\begin{pmatrix}
k_f & k_f & k_f & k_f \\
-k_f L_s & -k_f L_s & k_f L_s & k_f L_s \\
k_f L_s & -k_f L_s & -k_f L_s & k_f L_s \\
-k_m & k_m & -k_m & k_m
\end{pmatrix}}_{\mathbf{A}} \underbrace{\begin{pmatrix} \Omega_1^2 \\ \Omega_2^2 \\ \Omega_3^2 \\ \Omega_4^2 \end{pmatrix}}_{\boldsymbol{\Omega}^2}$$

The **inverse allocation** (mixer inverse) is:

$$\boldsymbol{\Omega}_{\text{des}}^2 = \mathbf{A}^{-1} \mathbf{w}_{\text{des}}$$

Since $\mathbf{A}$ is a $4 \times 4$ matrix with full rank (for $k_f > 0$, $k_m > 0$, $L > 0$), the inverse exists and is unique. This means the quadrotor system is **fully actuated** in the wrench space $(F, \tau_x, \tau_y, \tau_z)$ — any desired wrench can be uniquely decomposed into four motor speed commands (subject to positivity and saturation constraints).

**Analytical structure of $\mathbf{A}^{-1}$:** The matrix $\mathbf{A}$ has a particular structure due to the symmetry of the X-configuration. Defining $a = 1/(4k_f)$, $b = 1/(4k_f L_s)$, and $c = 1/(4k_m)$, the inverse can be written as:

$$\mathbf{A}^{-1} = \begin{pmatrix}
a & -b & b & -c \\
a & -b & -b & c \\
a & b & -b & -c \\
a & b & b & c
\end{pmatrix}$$

### D. Motor Speed Clamping

The desired motor speed squared $\Omega_{i,d}^2$ obtained from the mixer may be negative (indicating an infeasible wrench) or exceed the motor's maximum capacity. The clamping operation is:

$$\Omega_{i,d}^2 \leftarrow \text{clamp}(\Omega_{i,d}^2, \; 0, \; \Omega_{\max}^2)$$

$$\Omega_{i,d} = \sqrt{\Omega_{i,d}^2}$$

where $\Omega_{\max}$ is the maximum motor angular velocity. This clamping introduces a nonlinearity that is absent in the Direct Wrench Model and is critical for capturing realistic actuator saturation behavior.

**Saturation analysis:** At hover, each motor operates at:

$$\Omega_{\text{hover}} = \sqrt{\frac{mg}{4 k_f}} = \sqrt{\frac{1.08 \times 9.81}{4 \times 1.91 \times 10^{-6}}} \approx 1178 \; \text{rad/s}$$

With $\Omega_{\max} = 2500$ rad/s, the thrust-to-weight ratio is:

$$\frac{F_{\max}}{mg} = \frac{4 k_f \Omega_{\max}^2}{mg} = \frac{4 \times 1.91 \times 10^{-6} \times 2500^2}{1.08 \times 9.81} \approx 4.5$$

This provides a comfortable margin for aggressive maneuvers, consistent with typical racing quadrotors.

### E. First-Order Motor Dynamics

Real brushless DC motors exhibit a finite bandwidth due to electrical time constants (RL circuit of the windings), ESC (Electronic Speed Controller) update rate, and mechanical inertia of the rotor–propeller assembly. We model the aggregate effect as a first-order system:

$$\dot{\Omega}_i = \frac{\Omega_{i,d} - \Omega_i}{\tau_m}$$

where $\tau_m$ is the motor time constant. The transfer function from desired to actual motor speed is:

$$\frac{\Omega_i(s)}{\Omega_{i,d}(s)} = \frac{1}{\tau_m s + 1}$$

with a $-3$ dB bandwidth of $f_{-3\text{dB}} = 1/(2\pi\tau_m)$.

**Discrete-time integration:** Given the simulation timestep $\Delta t$, we employ a forward Euler discretization:

$$\Omega_i[k+1] = \Omega_i[k] + \alpha \left(\Omega_{i,d}[k] - \Omega_i[k]\right)$$

where $\alpha = \Delta t / \tau_m$ is the discrete-time filter coefficient. For the default parameters ($\Delta t = 3$ ms, $\tau_m = 20$ ms), $\alpha = 0.15$, yielding a stable and well-damped response.

**Stability condition:** The forward Euler method for the first-order ODE $\dot{x} = -(x - x_d)/\tau$ is stable if and only if $0 < \alpha < 2$, i.e., $\Delta t < 2\tau_m$. With the default parameters, $\Delta t / \tau_m = 0.15 \ll 2$, providing a large stability margin.

After integration, the motor speed is clamped to the physical range:

$$\Omega_i[k+1] \leftarrow \text{clamp}(\Omega_i[k+1], \; 0, \; \Omega_{\max})$$

**Step response characteristics:** For a step input from $\Omega_0$ to $\Omega_f$:

$$\Omega(t) = \Omega_f - (\Omega_f - \Omega_0) e^{-t/\tau_m}$$

- Time to reach 63%: $t_{63} = \tau_m = 20$ ms
- Time to reach 95%: $t_{95} = 3\tau_m = 60$ ms
- Time to reach 98%: $t_{98} = 4\tau_m = 80$ ms

For a typical motor (e.g., 2207 2450KV with 5-inch propellers), time constants between $15$ and $30$ ms are reported in the literature [4, 11].

**Default parameters:**

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Motor time constant | $\tau_m$ | $0.020$ s |
| Maximum motor speed | $\Omega_{\max}$ | $2500$ rad/s ($\approx 23\,870$ RPM) |

### F. Reconstruction of Actual Wrench

After the motor dynamics, the actual motor speeds $\Omega_i$ (which differ from the desired $\Omega_{i,d}$ due to the low-pass filtering effect) are used to compute the actual wrench via the forward allocation:

$$\mathbf{w}_{\text{act}} = \mathbf{A} \, \boldsymbol{\Omega}_{\text{act}}^2$$

Explicitly:

$$F_{\text{act}} = k_f \sum_{i=1}^{4} \Omega_i^2$$

$$\tau_{x,\text{act}} = k_f L_s \left(-\Omega_1^2 - \Omega_2^2 + \Omega_3^2 + \Omega_4^2\right)$$

$$\tau_{y,\text{act}} = k_f L_s \left(+\Omega_1^2 - \Omega_2^2 - \Omega_3^2 + \Omega_4^2\right)$$

$$\tau_{z,\text{act}} = k_m \left(-\Omega_1^2 + \Omega_2^2 - \Omega_3^2 + \Omega_4^2\right)$$

This actual wrench is then written to the MuJoCo actuators, replacing the desired wrench. The key insight is that the actual wrench will *lag* behind the desired wrench due to the motor dynamics, and may *differ in direction* due to the clamping nonlinearity.

### G. Aerodynamic Drag Model

Translational motion through air generates aerodynamic drag that opposes the vehicle's velocity. We employ a linear (viscous) drag model in the body frame:

$$\mathbf{f}_{\text{drag}}^B = -D_{\text{lin}} \, \mathbf{v}^B$$

where $D_{\text{lin}}$ [N/(m/s)] is the linear drag coefficient and $\mathbf{v}^B = \mathbf{R}^\top \mathbf{v}^W$ is the body-frame linear velocity.

The drag force affects the actuator outputs as follows:

- The $z$-component of drag modifies the effective thrust: $F_{\text{act}} \leftarrow F_{\text{act}} + f_{\text{drag},z}^B$
- The $x$ and $y$ components generate small parasitic torques through coupling with the center of pressure offset (modeled as a small effective moment arm of $\sim 0.01$ m):

$$\tau_{x,\text{act}} \leftarrow \tau_{x,\text{act}} + 0.01 \cdot f_{\text{drag},y}^B$$
$$\tau_{y,\text{act}} \leftarrow \tau_{y,\text{act}} + 0.01 \cdot f_{\text{drag},x}^B$$

**Justification for linear drag:** For the velocities typical of indoor quadrotor flight ($v < 5$ m/s), the Reynolds number is in the range $Re \approx 10^4$–$10^5$, where both linear and quadratic drag contributions are present. The linear model is adopted as a first-order approximation that captures the dominant effect without introducing additional parameters. For outdoor high-speed flight, a quadratic model $\mathbf{f}_{\text{drag}} \propto -\|\mathbf{v}\| \mathbf{v}$ would be more appropriate.

**Default parameter:**

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Linear drag coefficient | $D_{\text{lin}}$ | $0.1$ N/(m/s) |

### H. Complete Signal Flow

The complete computation within a single control cycle of the Motor-Level Model is:

1. Read current state from MuJoCo: $\mathbf{q}, \boldsymbol{\omega}^W, \mathbf{v}^W$
2. Transform to body frame: $\mathbf{R} \leftarrow \mathbf{R}(\mathbf{q})$, $\boldsymbol{\omega}^B \leftarrow \mathbf{R}^\top \boldsymbol{\omega}^W$
3. Rate controller: $\boldsymbol{\tau}_{\text{des}} \leftarrow \boldsymbol{\omega}^B \times (\mathbf{J}\boldsymbol{\omega}^B) - \mathbf{J}\mathbf{K}_\omega(\boldsymbol{\omega}^B - \boldsymbol{\omega}_d)$
4. Form desired wrench: $\mathbf{w}_{\text{des}} = (F_{\text{cmd}}, \tau_{x,\text{des}}, \tau_{y,\text{des}}, \tau_{z,\text{des}})^\top$
5. Inverse allocation: $\boldsymbol{\Omega}_{\text{des}}^2 \leftarrow \mathbf{A}^{-1} \mathbf{w}_{\text{des}}$
6. Clamp and extract speed: $\Omega_{i,d} \leftarrow \sqrt{\text{clamp}(\Omega_{i,d}^2, 0, \Omega_{\max}^2)}$
7. Motor dynamics: $\Omega_i \leftarrow \Omega_i + \alpha(\Omega_{i,d} - \Omega_i)$, clamp to $[0, \Omega_{\max}]$
8. Forward allocation: $\mathbf{w}_{\text{act}} \leftarrow \mathbf{A} \, \boldsymbol{\Omega}_{\text{act}}^2$
9. Add drag: modify $\mathbf{w}_{\text{act}}$ with drag forces
10. Write $\mathbf{w}_{\text{act}}$ to MuJoCo actuators

---

## VII. MuJoCo Physics Engine Configuration

### A. Integration Scheme

The simulator employs the **fourth-order Runge–Kutta (RK4)** integrator provided by MuJoCo. For a general ODE $\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, t)$, RK4 computes:

$$\mathbf{x}(t + \Delta t) = \mathbf{x}(t) + \frac{\Delta t}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)$$

where:

$$\mathbf{k}_1 = \mathbf{f}(\mathbf{x}, t), \quad \mathbf{k}_2 = \mathbf{f}\!\left(\mathbf{x} + \frac{\Delta t}{2}\mathbf{k}_1, t + \frac{\Delta t}{2}\right)$$

$$\mathbf{k}_3 = \mathbf{f}\!\left(\mathbf{x} + \frac{\Delta t}{2}\mathbf{k}_2, t + \frac{\Delta t}{2}\right), \quad \mathbf{k}_4 = \mathbf{f}\!\left(\mathbf{x} + \Delta t \, \mathbf{k}_3, t + \Delta t\right)$$

RK4 has a local truncation error of $\mathcal{O}(\Delta t^5)$ and a global error of $\mathcal{O}(\Delta t^4)$, providing excellent accuracy for the chosen timestep.

### B. Timestep Selection

The simulation timestep is $\Delta t = 3$ ms, selected as a compromise between:

- **Accuracy:** The fastest dynamics in the system are the motor dynamics with $\tau_m = 20$ ms. The ratio $\Delta t / \tau_m = 0.15$ ensures that the motor response is well-resolved (approximately $6.7$ steps per time constant).
- **Stability:** RK4 has a stability region in the complex $h\lambda$-plane that covers a substantial portion of the left half-plane. For the motor dynamics with eigenvalue $\lambda = -1/\tau_m = -50$ rad/s, $|h\lambda| = |0.003 \times (-50)| = 0.15$, well within the stability region.
- **Contact resolution:** MuJoCo's constraint solver benefits from small timesteps for accurate contact force computation. A 3 ms timestep corresponds to a simulation rate of approximately 333 Hz.
- **Real-time factor:** On modern hardware, the simulation achieves better than real-time performance at this timestep.

### C. Environmental Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Gravity | $(0, 0, -9.81)$ m/s² | Standard Earth gravity |
| Air density | $1.225$ kg/m³ | ISA sea level |
| Air viscosity | $1.8 \times 10^{-5}$ Pa·s | ISA at 15°C |

The air density and viscosity are used by MuJoCo for its internal fluid interaction models (when enabled). In the current configuration, these primarily affect the damping behavior of free joints.

### D. Joint Damping

The quadrotor's free joint includes a small artificial damping coefficient of $d_j = 0.001$ N·m·s/rad. This value is intentionally small and serves primarily to prevent numerical drift in the absence of active control, without significantly affecting the dynamics during controlled flight. Its effect can be quantified: at a typical angular velocity of $1$ rad/s, the damping torque is $10^{-3}$ N·m, which is three orders of magnitude smaller than the control torques ($\sim 0.1$–$0.3$ N·m).

---

## VIII. Software Architecture

### A. Plugin System

The simulator is built around MuJoCo's C++ plugin API, which provides three callback hooks:

1. **`init`**: Called once at model load. Reads configuration from XML, initializes ROS 2 nodes, and allocates resources.
2. **`compute`**: Called at every simulation step. Reads sensor data, processes control laws, and writes actuator commands.
3. **`destroy`**: Called at shutdown. Deallocates resources and shuts down ROS 2 nodes.

The plugin declares the capability flag `mjPLUGIN_ACTUATOR` and requires the velocity computation stage (`mjSTAGE_VEL`), ensuring that angular velocity data is available when `compute` is called.

### B. Plugin Ecosystem

The simulator employs four interacting plugins:

| Plugin | Type | Function |
|--------|------|----------|
| `AcroMode` | Actuator | Rate controller + optional motor model |
| `OdometryPublisher` | Sensor | Publishes pose and twist at 240 Hz |
| `ImuPublisher` | Sensor | Publishes accelerometer and gyroscope data at 200 Hz |
| `WrenchToActuators` | Actuator | Utility for wrench-based control |

### C. ROS 2 Interface

The AcroMode plugin instantiates an internal ROS 2 node that:

- **Subscribes** to `/<quad_name>/trpy_cmd` (`quadrotor_msgs/TRPYCommand`) for receiving thrust and angular velocity commands.
- Uses a **single-threaded executor** with non-blocking spin (`spin_once` with zero timeout) to process incoming messages within the MuJoCo simulation loop without introducing latency.

The sensor plugins publish:

- `/<quad_name>/odom` (`nav_msgs/Odometry`) at 240 Hz: full 6-DoF pose and twist.
- `/<quad_name>/imu` (`sensor_msgs/Imu`) at 200 Hz: 3-axis accelerometer and gyroscope readings.

### D. Configuration via XML

All simulator parameters are specified in XACRO (XML Macro) files that are processed at launch time. The use of XACRO allows:

- **Parameterization:** Motor model parameters, gains, and geometric properties are exposed as XACRO arguments.
- **Modularity:** Different quadrotor configurations (with/without motor model) are defined as separate macros that can be included in scene files.
- **Reproducibility:** The complete simulation configuration is captured in version-controlled XML files.

---

## IX. Outer-Loop Position Controller

To validate the simulator, an outer-loop PD position controller is implemented as a separate ROS 2 Python node. This controller operates at 100 Hz and computes thrust and angular velocity commands from position/velocity errors.

### A. Desired Acceleration

Given a target position $\mathbf{p}_d$ and current state $(\mathbf{p}, \mathbf{v})$, the desired acceleration in the world frame is:

$$\mathbf{a}_d = \begin{pmatrix} K_{p,xy}(p_{d,x} - p_x) - K_{d,xy} v_x \\ K_{p,xy}(p_{d,y} - p_y) - K_{d,xy} v_y \\ K_{p,z}(p_{d,z} - p_z) - K_{d,z} v_z + g \end{pmatrix}$$

where the gravitational term $g$ is added to the $z$-component to achieve hover at the setpoint.

### B. Thrust Computation

The thrust is the projection of the desired force onto the body $z$-axis:

$$F = m \, \mathbf{a}_d \cdot \mathbf{e}_3^B = m \, \mathbf{a}_d^\top \mathbf{R} \begin{pmatrix} 0 \\ 0 \\ 1 \end{pmatrix}$$

This projection naturally reduces thrust when the quadrotor is tilted, as only the vertical component of thrust counteracts gravity.

### C. Desired Attitude

The desired roll and pitch angles are extracted from the desired acceleration using the small-angle approximation:

$$\phi_d = \frac{a_{d,x} \sin\psi - a_{d,y} \cos\psi}{a_{d,z}}$$

$$\theta_d = \frac{a_{d,x} \cos\psi + a_{d,y} \sin\psi}{a_{d,z}}$$

where $\psi$ is the current yaw angle. These are clamped to $\pm 0.5$ rad ($\pm 28.6°$) for safety.

### D. Angular Velocity Commands

The angular velocity commands are proportional feedback on attitude error:

$$\omega_{d,x} = K_{p,\text{att}} (\phi_d - \phi)$$
$$\omega_{d,y} = K_{p,\text{att}} (\theta_d - \theta)$$
$$\omega_{d,z} = K_{p,\text{yaw}} \, \text{wrap}(\psi_d - \psi)$$

where $\text{wrap}(\cdot)$ constrains the yaw error to $(-\pi, \pi]$.

### E. Controller Gains

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Position XY proportional | $K_{p,xy}$ | $4.0$ s⁻² |
| Position XY derivative | $K_{d,xy}$ | $2.5$ s⁻¹ |
| Altitude proportional | $K_{p,z}$ | $8.0$ s⁻² |
| Altitude derivative | $K_{d,z}$ | $4.0$ s⁻¹ |
| Attitude proportional | $K_{p,\text{att}}$ | $6.0$ s⁻¹ |
| Yaw proportional | $K_{p,\text{yaw}}$ | $2.0$ s⁻¹ |

The damping ratios for the XY and Z channels are:

$$\zeta_{xy} = \frac{K_{d,xy}}{2\sqrt{K_{p,xy}}} = \frac{2.5}{2\sqrt{4.0}} = 0.625$$

$$\zeta_z = \frac{K_{d,z}}{2\sqrt{K_{p,z}}} = \frac{4.0}{2\sqrt{8.0}} = 0.707$$

Both are underdamped to slightly critically damped, providing a balance between fast response and overshoot suppression.

---

## X. Comparison of Actuation Models

### A. Structural Differences

| Aspect | Direct Wrench (Model I) | Motor-Level (Model II) |
|--------|------------------------|------------------------|
| Actuation path | $\mathbf{w}_{\text{des}} \to$ MuJoCo | $\mathbf{w}_{\text{des}} \to$ mixer $\to$ motor dyn. $\to$ propeller $\to$ MuJoCo |
| Motor state | None | $\boldsymbol{\Omega} \in \mathbb{R}^4$ |
| Bandwidth | Infinite (instantaneous) | $f_{-3\text{dB}} = 1/(2\pi\tau_m) \approx 8$ Hz |
| Saturation | Per-actuator clamping | Per-motor speed clamping + allocation coupling |
| Thrust–torque coupling | Independent | Coupled through shared motors |
| Aerodynamic drag | Not modeled | Linear body-frame drag |
| Computation cost | Negligible | 4×4 matrix multiply + 4 first-order ODEs |

### B. Transfer Function Analysis

For the Direct Wrench Model, the transfer function from desired to actual wrench is unity:

$$\frac{\mathbf{w}_{\text{act}}(s)}{\mathbf{w}_{\text{des}}(s)} = \mathbf{I}_4$$

For the Motor-Level Model, assuming operation in the linear region (no clamping), the transfer function introduces a first-order lag:

$$\frac{\mathbf{w}_{\text{act}}(s)}{\mathbf{w}_{\text{des}}(s)} \approx \frac{1}{\tau_m s + 1} \mathbf{I}_4$$

This approximation holds when the operating point is far from saturation and the wrench changes are small enough that the linearization of $\Omega_i \propto \sqrt{\Omega_i^2}$ is valid.

### C. Effect on Closed-Loop Stability

The introduction of actuator dynamics reduces the gain margin and phase margin of the closed-loop system. For the inner-loop rate controller, the open-loop transfer function becomes:

**Direct Wrench:**
$$G_{\text{OL},i}(s) = \frac{K_{\omega,i}}{s}$$

**Motor-Level:**
$$G_{\text{OL},i}(s) = \frac{K_{\omega,i}}{s(\tau_m s + 1)}$$

The additional pole at $s = -1/\tau_m$ reduces the phase margin by:

$$\Delta \phi = -\arctan(\omega_c \tau_m)$$

where $\omega_c$ is the crossover frequency. For the yaw axis ($K_{\omega,z} = 45$ rad/s, $\omega_c \approx 45$ rad/s, $\tau_m = 0.02$ s):

$$\Delta \phi = -\arctan(45 \times 0.02) = -\arctan(0.9) \approx -42°$$

This significant phase margin reduction may require re-tuning the rate controller gains when switching from the Direct Wrench to the Motor-Level Model, particularly for aggressive maneuvers.

---

## XI. Digital Twin Calibration Strategy

The Motor-Level Model is designed as a digital twin framework. The following calibration procedure is recommended:

### A. Inertial Properties

1. **Mass**: Weigh the complete drone with a precision scale.
2. **Inertia tensor**: Estimate using bifilar pendulum measurements or CAD model computation. Cross-validate with frequency response identification.

### B. Motor and Propeller Parameters

1. **$k_f$**: Mount the motor–propeller assembly on a thrust stand. Command various throttle levels and record thrust vs. RPM. Fit $f = k_f \Omega^2$.
2. **$k_m$**: Use a torque-measuring thrust stand, or estimate from the ratio $k_m/k_f \approx 0.01$–$0.02$ (propeller-dependent).
3. **$\tau_m$**: Apply a step command to the ESC and record the motor speed response with a high-frequency RPM sensor (e.g., optical tachometer at $>1$ kHz). Fit the exponential response to extract $\tau_m$.
4. **$\Omega_{\max}$**: Read from the motor datasheet or measure at maximum throttle with no load.

### C. Aerodynamic Drag

Estimate $D_{\text{lin}}$ from flight data by analyzing the deceleration profile during coast-down maneuvers (cutting throttle at constant velocity and measuring deceleration).

### D. Validation Procedure

1. Perform identical maneuvers on the physical drone and the simulator.
2. Compare state trajectories (position, velocity, attitude) using metrics such as RMSE and maximum tracking error.
3. Iteratively refine parameters to minimize the sim-to-real gap.

---

## XII. Simulation Considerations and Limitations

### A. Modeling Assumptions

1. **Rigid body**: The quadrotor frame is assumed perfectly rigid. Structural flexibility of the arms is not modeled.
2. **Symmetric inertia**: The inertia tensor is assumed diagonal, neglecting products of inertia.
3. **Constant propeller coefficients**: $k_f$ and $k_m$ are assumed constant, independent of advance ratio (valid in hover and slow flight).
4. **No ground effect**: The increase in thrust efficiency near the ground (typically within one rotor diameter) is not modeled.
5. **No blade flapping**: Rotor blade flapping effects, which introduce H-forces and pitch/roll moments at high forward speeds, are neglected.
6. **No inter-rotor aerodynamic interaction**: Downwash from one rotor affecting another is not captured.
7. **Linear drag**: A velocity-proportional drag model is used instead of the more physically accurate quadratic model.
8. **Ideal sensors**: The odometry and IMU outputs are noise-free and bias-free (directly from MuJoCo's state).
9. **No battery voltage drop**: Motor performance degradation due to battery discharge is not modeled.
10. **No ESC dynamics**: The ESC is assumed to have negligible latency; only the motor electromechanical dynamics are modeled.

### B. Numerical Considerations

1. **Motor dynamics discretization**: The forward Euler method used for motor dynamics integration has a first-order accuracy ($\mathcal{O}(\Delta t)$). While the overall MuJoCo integration uses RK4, the motor state is integrated internally in the plugin at each `compute` call. The error introduced by this mismatch is bounded by:

$$\epsilon_{\text{motor}} \leq \frac{\Delta t^2}{2\tau_m} (\Omega_{d} - \Omega) \leq \frac{(0.003)^2}{2 \times 0.02} \times 2500 \approx 0.56 \; \text{rad/s}$$

which is $0.02\%$ of $\Omega_{\max}$, negligible for practical purposes.

2. **Quaternion normalization**: MuJoCo internally maintains unit quaternions during integration, preventing the drift that affects Euler angle or rotation matrix propagation.

3. **Control–simulation rate mismatch**: The control rate (500 Hz) and simulation rate (~333 Hz) are not integer multiples of each other. The timing mechanism in the plugin handles this gracefully through the accumulator-based approach, but it means that the control update does not occur at exactly uniform intervals (jitter of up to $\pm \Delta t/2 = \pm 1.5$ ms). This is consistent with real-time systems where timer jitter is unavoidable.

### C. Computational Performance

The Motor-Level Model adds minimal computational overhead compared to the Direct Wrench Model. The additional operations per control step are:

- One $4 \times 4$ matrix–vector multiplication ($\mathbf{A}^{-1} \mathbf{w}$): 16 multiply-adds
- Four square root operations: 4 FLOPs
- Four first-order filter updates: 12 FLOPs
- One forward allocation ($\mathbf{A} \boldsymbol{\Omega}^2$): 16 multiply-adds
- Drag computation: 15 FLOPs (rotation, multiply, add)

Total: approximately 80 floating-point operations, which is negligible compared to MuJoCo's constraint solver.

---

## XIII. Conclusions

This paper has presented a rigorous mathematical formulation of a modular quadrotor simulator built on MuJoCo and ROS 2, implementing two actuation models of increasing fidelity:

1. The **Direct Wrench Model** provides a computationally efficient baseline that applies the desired thrust and torques directly to the rigid body, bypassing motor and propeller dynamics. This model is suitable for high-level control algorithm development where actuator bandwidth is not a limiting factor.

2. The **Motor-Level Model** introduces a realistic actuation pipeline that includes control allocation in Betaflight X-configuration, first-order motor electromechanical dynamics, propeller aerodynamic models ($f = k_f\Omega^2$, $\tau = k_m\Omega^2$), and body-frame aerodynamic drag. This model captures critical phenomena such as actuator lag, motor saturation, thrust–torque coupling, and the nonlinear mapping between desired and achievable wrenches.

Both models share a common inner-loop angular rate controller based on gyroscopic compensation and proportional feedback, providing a consistent interface to external controllers.

The framework is designed as a digital twin platform, where every physical parameter — from inertial properties to motor time constants — can be calibrated against a physical drone. The modular plugin architecture enables seamless switching between actuation fidelity levels and straightforward integration with advanced controllers such as NMPC.

Future work includes: (i) experimental validation against a physical Betaflight-based quadrotor, (ii) extension of the propeller model to include advance ratio effects and blade element theory, (iii) incorporation of sensor noise models for more realistic state estimation testing, (iv) addition of battery discharge dynamics, and (v) integration with Betaflight SITL for full flight stack-in-the-loop simulation.

---

## References

[1] E. Todorov, T. Erez, and Y. Tassa, "MuJoCo: A physics engine for model-based control," in *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 2012, pp. 5026–5033.

[2] R. Mahony, V. Kumar, and P. Corke, "Multirotor aerial vehicles: Modeling, estimation, and control of quadrotor," *IEEE Robotics & Automation Magazine*, vol. 19, no. 3, pp. 20–32, 2012.

[3] D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," in *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*, 2011, pp. 2520–2525.

[4] M. Faessler, D. Falanga, and D. Scaramuzza, "Thrust mixing, saturation, and body-rate control for accurate aggressive quadrotor flight," *IEEE Robotics and Automation Letters*, vol. 2, no. 2, pp. 476–482, 2017.

[5] G. Torrente, E. Kaufmann, P. Föhn, and D. Scaramuzza, "Data-driven MPC for quadrotors," *IEEE Robotics and Automation Letters*, vol. 6, no. 2, pp. 3769–3776, 2021.

[6] N. Koenig and A. Howard, "Design and use paradigms for Gazebo, an open-source multi-robot simulator," in *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 2004, pp. 2149–2154.

[7] F. Furrer, M. Burri, M. Achtelik, and R. Siegwart, "RotorS — A modular Gazebo MAV simulator framework," in *Robot Operating System (ROS)*, vol. 625, Springer, 2016, pp. 595–625.

[8] S. Shah, D. Dey, C. Lovett, and A. Kapoor, "AirSim: High-fidelity visual and physical simulation for autonomous vehicles," in *Field and Service Robotics*, Springer, 2018, pp. 621–635.

[9] P. Foehn, D. Bauer, E. Kaufmann, T. Cieslewski, and D. Scaramuzza, "Flightmare: A flexible quadrotor simulator," in *Proc. Conf. Robot Learning (CoRL)*, 2020.

[10] M. Grieves and J. Vickers, "Digital twin: Mitigating unpredictable, undesirable emergent behavior in complex systems," in *Transdisciplinary Perspectives on Complex Systems*, Springer, 2017, pp. 85–113.

[11] G. Shi, X. Shi, M. O'Connell, R. Yu, K. Azizzadenesheli, A. Anandkumar, Y. Yue, and S.-J. Chung, "Neural lander: Stable drone landing control using learned dynamics," in *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*, 2019.

---

## Appendix A: Allocation Matrix Derivation for Arbitrary Motor Configurations

For a general $n$-motor multirotor with motor $i$ located at body-frame position $(x_i, y_i, 0)$ and spinning with direction $\sigma_i \in \{+1, -1\}$ (CCW/CW), the allocation matrix row-by-row is:

$$A_{1,i} = k_f$$

$$A_{2,i} = -k_f \, y_i \quad \text{(roll torque from thrust offset in } y\text{)}$$

$$A_{3,i} = +k_f \, x_i \quad \text{(pitch torque from thrust offset in } x\text{)}$$

$$A_{4,i} = \sigma_i \, k_m$$

For the specific Betaflight X-configuration with $L_s = L/\sqrt{2}$:

| Motor | $x_i$ | $y_i$ | $\sigma_i$ |
|-------|--------|--------|-------------|
| M1 (FR, CW) | $+L_s$ | $-L_s$ | $-1$ |
| M2 (RR, CCW) | $-L_s$ | $-L_s$ | $+1$ |
| M3 (RL, CW) | $-L_s$ | $+L_s$ | $-1$ |
| M4 (FL, CCW) | $+L_s$ | $+L_s$ | $+1$ |

Substituting:

$$\mathbf{A} = \begin{pmatrix}
k_f & k_f & k_f & k_f \\
k_f L_s & k_f L_s & -k_f L_s & -k_f L_s \\
k_f L_s & -k_f L_s & -k_f L_s & k_f L_s \\
-k_m & k_m & -k_m & k_m
\end{pmatrix}$$

> **Note on sign convention:** The roll torque row uses $-y_i$ (positive roll is rotation about $+x$, which is generated by excess thrust on the $-y$ side). The sign convention matches the implementation in the simulator code.

---

## Appendix B: Parameter Summary

### B.1 Quadrotor Physical Parameters

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Total mass | $m$ | $1.05$ kg | MuJoCo model |
| Inertia $J_{xx}$ | — | $3.454 \times 10^{-3}$ kg·m² | MuJoCo auto-compute |
| Inertia $J_{yy}$ | — | $1.797 \times 10^{-3}$ kg·m² | MuJoCo auto-compute |
| Inertia $J_{zz}$ | — | $1.797 \times 10^{-3}$ kg·m² | MuJoCo auto-compute |
| Arm length | $L$ | $0.1$ m | Model geometry |
| Effective arm | $L_s$ | $0.0707$ m | $L/\sqrt{2}$ |

### B.2 Motor and Propeller Parameters

| Parameter | Symbol | Value | Units |
|-----------|--------|-------|-------|
| Thrust coefficient | $k_f$ | $1.91 \times 10^{-6}$ | N/(rad/s)² |
| Torque coefficient | $k_m$ | $2.6 \times 10^{-8}$ | N·m/(rad/s)² |
| Motor time constant | $\tau_m$ | $0.020$ | s |
| Max motor speed | $\Omega_{\max}$ | $2500$ | rad/s |
| Drag coefficient | $D_{\text{lin}}$ | $0.1$ | N/(m/s) |

### B.3 Controller Parameters

| Parameter | Symbol | Value | Units |
|-----------|--------|-------|-------|
| Roll rate gain | $K_{\omega,x}$ | $20.0$ | — |
| Pitch rate gain | $K_{\omega,y}$ | $35.0$ | — |
| Yaw rate gain | $K_{\omega,z}$ | $45.0$ | — |
| Control frequency | $f_c$ | $500$ | Hz |
| Simulation timestep | $\Delta t$ | $0.003$ | s |

### B.4 Operating Point at Hover

| Quantity | Expression | Value |
|----------|------------|-------|
| Hover thrust | $mg$ | $10.30$ N |
| Per-motor thrust | $mg/4$ | $2.58$ N |
| Hover motor speed | $\sqrt{mg/(4k_f)}$ | $1161$ rad/s ($\approx 11\,090$ RPM) |
| Hover motor speed ratio | $\Omega_{\text{hover}}/\Omega_{\max}$ | $46.4\%$ |
| Max total thrust | $4 k_f \Omega_{\max}^2$ | $47.75$ N |
| Thrust-to-weight ratio | $F_{\max}/(mg)$ | $4.64$ |

---

*Manuscript prepared for submission. © 2026.*
