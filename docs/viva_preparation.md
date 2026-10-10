# Lower-Limb Rehabilitation Exoskeleton: Viva Defense Guide & Demonstration Checklist

**Course:** BAEEE202 – Control Systems Engineering  
**Authors:** Madhumitha (25BEL0042) & Monica Shree (25BEL0039)  
**Evaluation:** Academic Evaluation (30 Marks)  

---

## 🎯 Quick Demonstration Checklist (5-Minute Evaluation Walkthrough)

When presenting to your evaluator or external examiner, follow this structured, confident sequence:

### Step 1: Launch the Master Simulation (Time: 30 seconds)
1. Open MATLAB.
2. In the Current Folder bar, navigate to:  
   `C:\Users\madhu\OneDrive\Documents\Lower-Limb Exoskeleton for Rehabilitation`
3. In the Command Window, run:
   ```matlab
   addpath('scripts');
   run_complete_project;
   ```
4. **Point out:** The script automatically executes all 6 scenarios, cross-validates Simulink models, compiles performance metrics, and exports all figures to the `results/` folder in ~1.5 minutes.

### Step 2: Show the Plant Model and Open-Loop Response (Time: 1 minute)
- **Show Figure 2 & Figure 3:** Open `results/fig02_open_loop_step_response.png` and `results/fig03_plant_bode_plot.png`.
- **Key Talking Points:**
  - *"Our system is a single-DOF rotational joint in the sagittal plane: $J\ddot{\theta} + B\dot{\theta} + K\theta = \tau$."*
  - *"The open-loop plant transfer function is $G(s) = \frac{1}{s^2 + 0.5s + 5}$. The poles are at $s = -0.25 \pm j2.222\text{ rad/s}$."*
  - *"Because the damping ratio is very low ($\zeta = 0.1118$), the open-loop step response has huge overshoot ($70.2\%$) and takes over 15 seconds to settle. This clearly proves why active closed-loop feedback control is required."*

### Step 3: Show Baseline PID vs Advanced Sliding Mode Control (Time: 1.5 minutes)
- **Show Figure 10:** Open `results/fig10_pid_vs_smc_nominal.png`.
- **Key Talking Points:**
  - *"We tuned a classical PID controller ($K_p = 42.282, K_i = 35.724, K_d = 9.807$). For a static step input, it performs well ($T_s = 2.35\text{ s}, \text{OS} = 13.8\%$)."*
  - *"However, when tracking a dynamic $0.5\text{ Hz}$ physiological gait trajectory, PID exhibits phase lag and high tracking error ($\text{RMSE} = 16.41^\circ, \text{MAE} = 14.64^\circ$) because it lacks dynamic feedforward."*
  - *"Our Sliding Mode Controller (SMC) incorporates equivalent model-based feedforward $\tau_{\text{eq}} = J(\ddot{\theta}_d + \lambda\dot{e}) + B\dot{\theta} + K\theta$ plus a robust switching term $\tau_{\text{sw}} = K_{\text{sw}}\tanh(s/\Phi)$."*
  - *"SMC cuts Mean Absolute Error by $89.1\%$ (down to $1.60^\circ$) and RMSE by $60.6\%$ (down to $6.46^\circ$) while consuming almost identical actuator energy ($139.3\text{ N}^2\text{m}^2\text{s}$ vs $137.6\text{ N}^2\text{m}^2\text{s}$)."*

### Step 4: Show Disturbance Rejection (Time: 1 minute)
- **Show Figure 11 & Figure 8:** Open `results/fig11_pid_vs_smc_disturbance.png`.
- **Key Talking Points:**
  - *"At $t = 5\text{ s}$ to $6\text{ s}$, we inject a $5.0\text{ N}\cdot\text{m}$ resistive torque disturbance representing an involuntary patient muscle spasm or spasticity."*
  - *"Under this disturbance, PID deviates by $24.60^\circ$ and takes $0.638\text{ s}$ to recover."*
  - *"SMC completely crushes the disturbance, keeping peak deviation down to just $0.21^\circ$ ($99.1\%$ suppression) and recovers in $1\text{ ms}$ because our switching gain $K_{\text{sw}} = 10.0\text{ N}\cdot\text{m}$ is strictly greater than the $5.0\text{ N}\cdot\text{m}$ disturbance."*

### Step 5: Show Assist-as-Needed (AAN) Adaptation (Time: 1 minute)
- **Show Figure 9:** Open `results/fig09_aan_assistance_adaptation.png`.
- **Key Talking Points:**
  - *"In neurorehabilitation, 100% rigid guidance causes 'learned helplessness'. Our Assist-as-Needed algorithm dynamically adapts assistance $\alpha(t) \in [0.20, 1.00]$ based on filtered absolute error across $[2^\circ, 10^\circ]$ thresholds."*
  - *"During the disturbance, $\alpha(t)$ automatically surges to $1.00$ ($100\%$ assistance) to ensure patient safety, and relaxes when the patient is tracking well."*

---

## 📚 35 Comprehensive Viva Questions and Answers

### Section 1: Project Motivation, Clinical Context & Biomechanics

#### Q1: What is the clinical motivation for developing a lower-limb rehabilitation exoskeleton?
**Answer:** Neurological trauma such as ischemic stroke or spinal cord injury impairs motor control pathways, leading to hemiparesis or muscle weakness. Traditional physical therapy requires two to three therapists to manually move the patient's legs on a treadmill, which is physically exhausting and difficult to standardize. Robotic exoskeletons provide repeatable, programmable joint assistance over thousands of gait cycles, promoting neuroplasticity—the brain's ability to rewire motor pathways through repetitive task-specific practice.

#### Q2: Why is trajectory tracking necessary in robotic gait rehabilitation?
**Answer:** Healthy human locomotion relies on precise multi-joint kinematic coordination. If a patient is allowed to walk with abnormal kinematics, they develop compensatory gait habits (such as hip hiking or circumduction) that cause joint degeneration and increase fall risk. Trajectory tracking gently guides the limb along normal physiological kinematic curves (e.g. $0^\circ$ to $60^\circ$ knee flexion-extension) to retrain proper locomotor patterns.

#### Q3: What does the desired gait trajectory equation $\theta_d(t) = 30 + 30\sin(2\pi \cdot 0.5 \cdot t)$ represent physically?
**Answer:** It models periodic sagittal-plane knee/hip flexion and extension:
- $f = 0.5\text{ Hz}$ represents standard rehabilitation walking cadence ($1\text{ stride every } 2.0\text{ seconds}$, or $30\text{ strides/min}$).
- The offset $\theta_0 = 30^\circ$ combined with amplitude $A = 30^\circ$ produces a range of motion from $0^\circ$ (full extension at heel-strike/stance) to $60^\circ$ (peak flexion during swing phase), mirroring normal human knee sagittal kinematics.

#### Q4: Why did you derive exact analytical derivatives instead of using numerical differentiation (`diff`) for reference velocity and acceleration?
**Answer:** Finite-difference numerical differentiation introduces two major errors:
1. It amplifies high-frequency numerical discretization noise by a factor of $1/\Delta t$ ($1000\times$ for $T_s = 0.001\text{ s}$).
2. It causes boundary shrinkage (array size decreases by 1 each time) or endpoint edge distortions.
By differentiating analytically:
$$\dot{\theta}_d(t) = A (2\pi f) \cos(2\pi f t), \quad \ddot{\theta}_d(t) = -A (2\pi f)^2 \sin(2\pi f t)$$
we obtain perfectly smooth, noise-free feedforward velocity and acceleration references.

#### Q5: Is this system ready for clinical testing on human subjects?
**Answer:** Absolutely not. This is an academic simulation project designed to evaluate control algorithms. A real clinical exoskeleton requires ISO 13482 safety compliance, mechanical limit stops, redundant torque sensors, emergency electrical cutoffs, zero-backlash series elastic actuators, and clinical ethics approval.

---

### Section 2: Mathematical Modelling & System Dynamics

#### Q6: What is the governing differential equation of the exoskeleton plant?
**Answer:** The single-joint rotational plant in the sagittal plane is modeled by:
$$J \ddot{\theta}(t) + B \dot{\theta}(t) + K \theta(t) = \tau(t) + \tau_{\text{dist}}(t)$$
where $\tau(t)$ is actuator torque and $\tau_{\text{dist}}(t)$ is external interaction torque.

#### Q7: What are the physical meanings and units of $J$, $B$, and $K$?
**Answer:**
- $J = 1.0\text{ kg}\cdot\text{m}^2$: Equivalent rotational moment of inertia (representing the combined mass distribution of the human shank/foot and exoskeleton mechanical links rotated about the joint axis).
- $B = 0.5\text{ N}\cdot\text{m}\cdot\text{s/rad}$: Viscous damping coefficient (representing joint fluid shear resistance, soft-tissue friction, and motor bearing damping).
- $K = 5.0\text{ N}\cdot\text{m/rad}$: Passive torsional stiffness (representing anatomical ligament elasticity, tendon tension, and joint restoring torque).

#### Q8: How is the open-loop transfer function derived from the differential equation?
**Answer:** Assuming zero initial conditions and applying the Laplace transform:
$$\mathcal{L}\{J\ddot{\theta} + B\dot{\theta} + K\theta\} = (J s^2 + B s + K) \Theta(s) = \mathcal{T}(s)$$
$$G(s) = \frac{\Theta(s)}{\mathcal{T}(s)} = \frac{1}{J s^2 + B s + K} = \frac{1}{s^2 + 0.5 s + 5.0}$$

#### Q9: What are the poles of the plant and what do they tell you about stability?
**Answer:** The poles are the roots of $s^2 + 0.5s + 5 = 0$:
$$s_{1,2} = -0.25 \pm j2.2220\text{ rad/s}$$
Because the real parts are negative ($\text{Re}(s) = -0.25 < 0$), both poles lie in the left half of the complex plane (LHP). Therefore, the open-loop plant is Bounded-Input Bounded-Output (BIBO) stable.

#### Q10: What are the natural frequency and damping ratio of the plant?
**Answer:** Comparing to standard second-order form $s^2 + 2\zeta\omega_n s + \omega_n^2$:
- Undamped natural frequency: $\omega_n = \sqrt{K/J} = \sqrt{5} \approx 2.2361\text{ rad/s}$.
- Damping ratio: $\zeta = \frac{B}{2\sqrt{J K}} = \frac{0.5}{2\sqrt{5}} \approx 0.1118$.
Because $\zeta < 1$ (and specifically $\zeta \ll 0.707$), the system is severely underdamped, explaining the large $70.2\%$ overshoot and prolonged $15.63\text{ s}$ settling time observed in the open-loop step response.

#### Q11: What is the state-space representation of the plant?
**Answer:** Choosing state variables $x_1 = \theta$ and $x_2 = \dot{\theta}$:
$$\dot{x} = \begin{bmatrix} 0 & 1 \\ -5.0 & -0.5 \end{bmatrix} x + \begin{bmatrix} 0 \\ 1.0 \end{bmatrix} u, \quad y = \begin{bmatrix} 1 & 0 \end{bmatrix} x$$
The controllability matrix $\mathcal{C} = [B, AB]$ and observability matrix $\mathcal{O} = [C; CA]$ both have rank 2, confirming full controllability and full observability.

---

### Section 3: Classical Feedback Control & PID Design

#### Q12: Why did we convert the desired gait trajectory from degrees to radians before feeding it to the controller?
**Answer:** The plant's physical parameters ($J$, $B$, $K$) are formulated in standard SI metric units ($\text{kg}\cdot\text{m}^2$, $\text{N}\cdot\text{m}\cdot\text{s/rad}$, $\text{N}\cdot\text{m/rad}$), where angles are in radians. If an angle in degrees ($30^\circ$) were fed directly without conversion, the controller would interpret it as $30\text{ radians} \approx 1718^\circ$, applying an enormous $57.3\times$ erroneous torque that would immediately saturate and destabilize the simulation. Therefore, all internal dynamic equations use radians, and values are converted to degrees only for user visualization.

#### Q13: What roles do the $K_p$, $K_i$, and $K_d$ terms play in controlling the exoskeleton?
**Answer:**
- $K_p$ (Proportional): Acts like a virtual restoring spring. As error increases, it applies proportional corrective torque.
- $K_i$ (Integral): Accumulates steady error over time to eliminate offsets caused by passive joint stiffness ($K\theta$) or constant gravitational load.
- $K_d$ (Derivative): Acts like virtual viscous damping, predicting error velocity and opposing rapid changes to prevent overshoot.

#### Q14: What PID gains were tuned, and what step response performance did they achieve?
**Answer:** Tuned via MATLAB's `pidtune(G, 'PID')`:
- $K_p = 42.282161\text{ N}\cdot\text{m/rad}$
- $K_i = 35.723521\text{ N}\cdot\text{m/(rad}\cdot\text{s)}$
- $K_d = 9.806628\text{ N}\cdot\text{m}\cdot\text{s/rad}$
Under a $1.0\text{ rad}$ unit step test:
- Rise time: $0.1190\text{ s}$
- Settling time: $2.3497\text{ s}$
- Overshoot: $13.80\%$

#### Q15: Why is derivative filtering ($N = 100\text{ rad/s}$) necessary in the PID controller?
**Answer:** An ideal derivative term $D(s) = K_d s$ has infinite gain at infinite frequency. In real digital systems, measurement noise from joint encoders has high-frequency components that would cause the actuator to chatter wildly. Adding a first-order low-pass filter $D(s) = \frac{K_d N s}{s + N}$ limits high-frequency gain to $K_d N$, filtering out noise while preserving derivative damping.

#### Q16: What is integrator windup and how is it prevented?
**Answer:** When the actuator saturates at its maximum torque limit ($\pm 50\text{ N}\cdot\text{m}$), the error cannot be immediately reduced. A standard integrator continues accumulating error, building up an enormous integral term ("windup"). Once the setpoint is reached, the controller remains saturated in the wrong direction, causing massive overshoot. We implemented anti-windup clamping: whenever torque saturates and error has the same sign, integral accumulation is suspended.

#### Q17: Why does the tuned PID controller show high tracking error ($16.41^\circ$ RMSE) during dynamic gait tracking despite performing well on the step test?
**Answer:** A unit-step reference is static after $t = 0$. In contrast, the gait trajectory is constantly moving at $0.5\text{ Hz}$. A standard feedback PID controller is purely reactive—it can only generate corrective torque *after* an error has already formed. Because the plant has significant inertia ($J = 1.0$) and passive stiffness ($K = 5.0$), reactive feedback suffers from dynamic phase lag, resulting in large tracking errors during continuous motion.

---

### Section 4: Advanced Nonlinear Control (Sliding Mode Control)

#### Q18: What is Sliding Mode Control (SMC) and what is its main advantage?
**Answer:** SMC is a robust nonlinear variable-structure control technique. It defines a mathematical surface in error state space called the sliding manifold ($s = 0$). By driving system states onto this surface and keeping them there, the system's order is reduced and its closed-loop behavior becomes completely invariant to matched external disturbances and plant parameter uncertainties.

#### Q19: What is the sliding surface chosen for this second-order system?
**Answer:** A linear Hurwitz sliding surface:
$$s(t) = \dot{e}(t) + \lambda e(t)$$
where $e = \theta_d - \theta$ and $\lambda = 8.0\text{ s}^{-1}$. Once the system is on the surface ($s = 0$), the error dynamics satisfy $\dot{e} + \lambda e = 0$, guaranteeing that tracking error decays exponentially to zero at rate $e^{-\lambda t}$ regardless of disturbances.

#### Q20: What is the equivalent control torque ($\tau_{\text{eq}}$) and how is it derived?
**Answer:** Setting the time derivative of the sliding surface to zero ($\dot{s} = 0$) under nominal disturbance-free conditions:
$$\dot{s} = \ddot{\theta}_d - \ddot{\theta} + \lambda \dot{e} = 0$$
Substituting $\ddot{\theta} = \frac{1}{J}(\tau - B\dot{\theta} - K\theta)$ yields:
$$\tau_{\text{eq}} = J(\ddot{\theta}_d + \lambda \dot{e}) + B\dot{\theta} + K\theta$$
This term provides analytical feedforward: it cancels the plant's passive damping and stiffness and injects the exact acceleration torque required to follow $\ddot{\theta}_d$.

#### Q21: What is the switching control term ($\tau_{\text{sw}}$) and why is $K_{\text{sw}}$ chosen as $10.0\text{ N}\cdot\text{m}$?
**Answer:** The switching term $\tau_{\text{sw}}$ forces state trajectories onto the sliding manifold even when external disturbances $\tau_{\text{dist}}$ are present. By Lyapunov stability analysis with $V = \frac{1}{2} J s^2$:
$$\dot{V} \le -(K_{\text{sw}} - |\tau_{\text{dist}}|) |s|$$
To guarantee that $\dot{V} < 0$, the switching gain must satisfy $K_{\text{sw}} > \max |\tau_{\text{dist}}|$. Because our disturbance is $5.0\text{ N}\cdot\text{m}$, we selected $K_{\text{sw}} = 10.0\text{ N}\cdot\text{m}$, providing a robust margin of $\eta = 5.0\text{ N}\cdot\text{m}$.

#### Q22: What is chattering, why is it dangerous in rehabilitation, and how did you prevent it?
**Answer:** Ideal SMC uses a discontinuous signum function $\text{sgn}(s)$, which switches infinitely fast across $s = 0$. In physical systems with finite sampling rates, this causes high-frequency torque oscillations known as chattering. In an exoskeleton, chattering causes extreme mechanical vibration, wears out gearboxes, and could tear spastic human muscle tissues. We eliminated chattering by replacing $\text{sgn}(s)$ with a continuous boundary layer using the hyperbolic tangent function $\tanh(s / \Phi)$ with boundary layer thickness $\Phi = 0.05\text{ rad/s}$.

#### Q23: How does the phase portrait ($\dot{e}$ vs $e$) validate the Sliding Mode Controller?
**Answer:** In `results/fig07_smc_phase_portrait.png`, the phase portrait shows the error state trajectory starting at $(0, 0)$ and rapidly converging toward the straight line $\dot{e} = -\lambda e$ (the sliding line $s = 0$). Once on this line, the states remain tightly trapped within the boundary layer throughout the simulation, confirming Lyapunov reachability.

---

### Section 5: Disturbance Modelling & Assist-as-Needed (AAN)

#### Q24: How is the human–robot interaction disturbance modeled?
**Answer:** As a deterministic external torque pulse applied in the sagittal plane:
$$\tau_{\text{dist}}(t) = 5.0\text{ N}\cdot\text{m} \quad \text{for } t \in [5.0\text{ s}, 6.0\text{ s}]$$
This represents an involuntary muscle spasm, spastic reflex, or voluntary resistance opposing joint extension during mid-swing phase.

#### Q25: How did PID and SMC compare when rejecting this disturbance?
**Answer:**
- Under PID, the joint was pulled off course by up to $24.60^\circ$ of peak error, requiring $0.638\text{ s}$ to recover after the disturbance ended.
- Under SMC, the peak error during the disturbance was only $0.21^\circ$ ($99.1\%$ suppression), and recovery took $0.001\text{ s}$ ($1\text{ ms}$). This demonstrates the immense robust disturbance rejection capability of Sliding Mode Control.

#### Q26: What is the concept of Assist-as-Needed (AAN) in neurorehabilitation?
**Answer:** If an exoskeleton moves a patient's leg with 100% rigid authority, the patient's nervous system realizes it does not need to exert effort. This phenomenon—learned helplessness—stops motor cortex recovery. AAN dynamically reduces robot intervention when the patient tracks well, forcing biological motor units to fire, but intervenes when tracking error exceeds safe physiological limits.

#### Q27: How is your AAN assistance factor $\alpha(t)$ calculated?
**Answer:** It uses a piecewise linear mapping based on low-pass filtered absolute tracking error $e_{\text{smooth}}(t)$:
- If $e_{\text{smooth}} \le 2.0^\circ$: $\alpha = 0.20$ ($20\%$ baseline assistance).
- If $2.0^\circ < e_{\text{smooth}} < 10.0^\circ$: $\alpha$ scales linearly between $0.20$ and $1.00$.
- If $e_{\text{smooth}} \ge 10.0^\circ$: $\alpha = 1.00$ ($100\%$ full assistance).
A first-order lag filter ($\tau = 0.05\text{ s}$) ensures smooth transitions and prevents torque discontinuities.

#### Q28: What happened to the assistance factor during the disturbance at $t = 5\text{ s}$?
**Answer:** When the $5\text{ N}\cdot\text{m}$ disturbance hit at $t = 5\text{ s}$, the tracking error rose rapidly above the $10^\circ$ threshold. The AAN algorithm immediately responded by surging $\alpha(t)$ to $1.00$ ($100\%$ assistance), delivering full corrective torque to reject the disturbance and protect the simulated patient.

---

### Section 6: Results, Metrics & Software Architecture

#### Q29: What performance metrics were computed and why?
**Answer:**
1. **RMSE (Root Mean Square Error):** Penalizes large deviations heavily, reflecting overall trajectory consistency.
2. **MAE (Mean Absolute Error):** Measures average tracking deviation in degrees.
3. **Maximum Absolute Error:** Identifies the worst-case instantaneous deviation.
4. **Control Energy ($\int \tau^2 dt$):** Measures electrical actuator effort and thermal dissipation.
5. **Disturbance Recovery Time:** Measures how quickly the closed-loop system re-enters the nominal error band after a disturbance.

#### Q30: Summarize the quantitative results: why is SMC superior to PID?
**Answer:**
- **MAE:** SMC achieved $1.60^\circ$ vs $14.64^\circ$ for PID (**$89.1\%$ reduction**).
- **RMSE:** SMC achieved $6.46^\circ$ vs $16.41^\circ$ for PID (**$60.6\%$ reduction**).
- **Disturbance Deviation:** SMC allowed only $0.21^\circ$ vs $24.60^\circ$ for PID (**$99.1\%$ suppression**).
- **Energy:** SMC used virtually identical energy ($139.30$ vs $137.55\text{ N}^2\cdot\text{m}^2\cdot\text{s}$).
SMC is superior because it uses model-based equivalent feedforward to cancel passive joint dynamics and a high-gain switching term to instantly reject external disturbances.

#### Q31: How do your MATLAB scripts and Simulink models communicate and cross-validate?
**Answer:** Central parameters are defined in `scripts/project_parameters.m`. The MATLAB scripts run discrete ODE integration, while the Simulink models (`exoskeleton_plant.slx` and `exoskeleton_gait_system.slx`) execute the same differential equations using Simulink's continuous/discrete solver (`ode4` Runge-Kutta). In `scripts/run_complete_project.m`, both Simulink models are simulated via the `sim` command, and logged signals match the script results with $< 10^{-6}\text{ rad}$ error.

#### Q32: What are the main limitations of this single-joint model?
**Answer:**
1. It is a 1-DOF model; it ignores dynamic coupling between the thigh and shank (Coriolis and centrifugal cross-terms from 2-DOF Euler-Lagrange equations).
2. It assumes linear stiffness and damping, whereas biological muscle-tendon complexes have highly nonlinear, state-dependent viscoelasticity.
3. It assumes rigid mechanical coupling without soft-tissue strap compliance or migration.

#### Q33: If you had another semester to expand this project, what would you add?
**Answer:**
1. Expand to a 2-DOF coupled hip-knee exoskeleton model using full nonlinear Euler-Lagrange dynamics:
   $$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau + \tau_{\text{int}}$$
2. Implement Surface Electromyography (sEMG) bio-signal processing to predict user movement intent before joint motion begins.
3. Design an Admittance/Impedance controller with an inner torque loop for compliant physical human–robot interaction.

#### Q34: What files were created and where are they located in the repository?
**Answer:**
- `scripts/project_parameters.m`: Central parameters and path initialization.
- `trajectories/generate_gait.m`: Analytical position, velocity, and acceleration reference generator.
- `models/exoskeleton_model.m`: Continuous transfer function, poles, Bode, and state space.
- `models/disturbance_model.m`: Human-robot interaction torque profile.
- `models/exoskeleton_plant.slx`: Step-response Simulink model with Outports.
- `models/exoskeleton_gait_system.slx`: Full dynamic gait tracking Simulink model.
- `controllers/design_pid.m`: Tuned PID with anti-windup, derivative filter, and dynamic tracking.
- `controllers/design_smc.m`: Robust Sliding Mode Control with boundary layer smoothing.
- `controllers/design_aan.m`: Assist-as-Needed adaptive control with error filtering.
- `controllers/compare_controllers.m`: Side-by-side benchmark runner.
- `scripts/run_complete_project.m`: Master automation script running all scenarios.
- `docs/technical_report.md`: 30-section comprehensive academic report.
- `docs/viva_preparation.md`: 35 questions and demonstration guide.
- `results/`: 12 high-resolution PNG figures, CSV metrics table, TXT summary, and MAT workspace.

#### Q35: What is the single most important control engineering lesson learned from this project?
**Answer:** That classical reactive feedback control (like PID) is fundamentally limited when tracking dynamic trajectories with passive plant stiffness, because it requires error to exist before producing corrective action. Incorporating model-based feedforward and robust sliding manifolds dramatically improves tracking accuracy and disturbance rejection without requiring excess actuator energy.
