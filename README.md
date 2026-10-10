# Adaptive Trajectory Control of a Lower-Limb Rehabilitation Exoskeleton for Human–Robot Interaction

[![MATLAB](https://img.shields.io/badge/MATLAB-R2026b-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Integrated-blue.svg)](https://www.mathworks.com/products/simulink.html)
[![Course](https://img.shields.io/badge/Course-BAEEE202%20Control%20Systems-green.svg)]()
[![Status](https://img.shields.io/badge/Simulation-Verified%20%26%20Reproducible-brightgreen.svg)]()

**Course Code:** BAEEE202 – Control Systems (Semester 3, B.Tech ECE)  
**Authors:** Madhumitha (25BEL0042) & Monica Shree (25BEL0039)  
**Repository:** [MadhumithaS2401/lower-limb-exoskeleton-control](https://github.com/MadhumithaS2401/lower-limb-exoskeleton-control)  
**Academic Weightage:** 30 Marks  

---

## 📌 Project Overview

This repository contains a complete, verified, and reproducible MATLAB & Simulink control systems simulation for an active **Lower-Limb Rehabilitation Exoskeleton** joint operating in the sagittal plane. 

The system models the rotational dynamics of a human–robot knee/hip joint subjected to passive viscoelastic resistance, actuator saturation, and external human–robot interaction disturbance torques (simulating spasticity or involuntary muscle contractions). The project implements and benchmarks:
1. **Mathematical Rotational Modeling:** Continuous-time transfer function and state-space modal analysis ($J = 1.0\text{ kg}\cdot\text{m}^2, B = 0.5\text{ N}\cdot\text{m}\cdot\text{s/rad}, K = 5.0\text{ N}\cdot\text{m/rad}$).
2. **Analytical Gait Kinematics:** Noise-free analytical physiological knee flexion-extension reference trajectory ($0.5\text{ Hz}$ cadence, $0^\circ$ to $60^\circ$ ROM).
3. **Classical Feedback Control:** Loop-shaped tuned PID controller ($K_p = 42.282, K_i = 35.724, K_d = 9.807$) with derivative filtering ($N = 100\text{ rad/s}$) and anti-windup clamping.
4. **Advanced Robust Control:** Lyapunov-based Sliding Mode Control (SMC) with equivalent dynamic feedforward, Hurwitz sliding surface ($\lambda = 8.0\text{ s}^{-1}$), and continuous boundary layer smoothing ($\Phi = 0.05\text{ rad/s}$) to suppress chattering.
5. **Human–Robot Interaction Disturbance:** Deterministic $5.0\text{ N}\cdot\text{m}$ resistive torque pulse applied from $t = 5.0\text{ s}$ to $6.0\text{ s}$.
6. **Assist-as-Needed (AAN) Adaptation:** Bounded error-dependent assistance scaling $\alpha(t) \in [0.20, 1.00]$ with first-order error filtering across $[2^\circ, 10^\circ]$ error thresholds.
7. **Simulink Integration:** Dual block diagram models (`exoskeleton_plant.slx` and `exoskeleton_gait_system.slx`) cross-validated with MATLAB discrete solvers.

---

## 📁 Repository Structure

```text
Lower-Limb Exoskeleton for Rehabilitation/
├── controllers/
│   ├── design_pid.m              # Tuned PID controller with step validation & gait tracking
│   ├── design_smc.m              # Robust Sliding Mode Controller with boundary layer
│   ├── design_aan.m              # Bounded Assist-as-Needed adaptive control algorithm
│   └── compare_controllers.m     # Rigorous side-by-side benchmark runner (PID vs SMC)
├── docs/
│   ├── technical_report.md       # Comprehensive 30-section academic project report
│   └── viva_preparation.md      # 35 viva defense questions & answers + demonstration guide
├── models/
│   ├── exoskeleton_model.m       # Transfer function, poles, Bode, and state-space analysis
│   ├── disturbance_model.m       # Human-robot interaction torque pulse generator
│   ├── exoskeleton_plant.slx     # Closed-loop step validation Simulink model with Outports
│   └── exoskeleton_gait_system.slx # Complete dynamic gait tracking Simulink system with scopes
├── results/                      # Auto-generated simulation artifacts
│   ├── fig01_desired_gait_trajectory.png
│   ├── fig02_open_loop_step_response.png
│   ├── fig03_plant_bode_plot.png
│   ├── fig04_pid_step_response.png
│   ├── fig05_pid_gait_tracking.png
│   ├── fig06_smc_gait_tracking.png
│   ├── fig07_smc_phase_portrait.png
│   ├── fig08_disturbance_profile.png
│   ├── fig09_aan_assistance_adaptation.png
│   ├── fig10_pid_vs_smc_nominal.png
│   ├── fig11_pid_vs_smc_disturbance.png
│   ├── fig12_metrics_comparison_barchart.png
│   ├── simulation_metrics_table.csv  # Consolidated quantitative performance metrics
│   ├── simulation_summary.txt        # Detailed human-readable text report
│   └── simulation_data.mat           # Complete workspace dataset for post-processing
├── scripts/
│   ├── project_parameters.m      # Centralized global parameters & path configuration
│   ├── build_simulink_model.m    # Programmatic generator for exoskeleton_gait_system.slx
│   └── run_complete_project.m    # Master automated entry-point runner (Scenarios A to F)
├── trajectories/
│   └── generate_gait.m           # Analytical gait position, velocity, and acceleration generator
├── .gitignore                    # Git rules ignoring temporary cache & autosaves
└── README.md                     # Project overview and reproduction instructions
```

---

## 🚀 Quickstart Guide (How to Run in MATLAB)

### Prerequisites:
- MATLAB (R2026b or R2022b+)
- Simulink
- Control System Toolbox

### Execution Steps:
1. Open MATLAB.
2. In the MATLAB Address Bar / Current Folder window, navigate to the project directory:
   ```matlab
   cd('C:\Users\madhu\OneDrive\Documents\Lower-Limb Exoskeleton for Rehabilitation')
   ```
3. To run the complete project end-to-end, simply execute:
   ```matlab
   addpath('scripts');
   run_complete_project;
   ```
4. The master runner will execute all 6 scenarios, cross-validate Simulink models, print formatted tables, and save all 12 figures and CSV tables to the `results/` folder.

---

## 📊 Verified Simulation Results

The following table summarizes the actual numerical results generated during the master simulation run (`results/simulation_metrics_table.csv`):

| Scenario | Controller Architecture | Disturbance Condition | Assistance Level | RMSE ($^\circ$) | MAE ($^\circ$) | Max Error ($^\circ$) | Control Energy ($\text{N}^2\cdot\text{m}^2\cdot\text{s}$) | Dist. Peak Error ($^\circ$) | Recovery Time ($s$) |
| :--- | :--- | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Scenario A** | Open-Loop Plant | None | N/A | — | — | — | $0.00$ | — | — |
| **Scenario B** | Tuned PID (Step) | None | $100\%$ | — | — | — | — | — | — |
| **Scenario C** | Baseline PID | None | $100\%$ | $16.409$ | $14.643$ | $33.839$ | $137.55$ | — | — |
| **Scenario D** | Sliding Mode (SMC) | None | $100\%$ | $\mathbf{6.458}$ | $\mathbf{1.598}$ | $34.199$ | $139.30$ | — | — |
| **Scenario E-1** | Baseline PID | $5.0\text{ N}\cdot\text{m}$ ($5-6\text{s}$) | $100\%$ | $16.423$ | $14.648$ | $33.839$ | $125.66$ | $24.599$ | $0.638$ |
| **Scenario E-2** | Sliding Mode (SMC) | $5.0\text{ N}\cdot\text{m}$ ($5-6\text{s}$) | $100\%$ | $\mathbf{6.458}$ | $\mathbf{1.617}$ | $34.199$ | $121.92$ | $\mathbf{0.214}$ | $\mathbf{0.001}$ |
| **Scenario F-1** | Fixed Baseline PID | $5.0\text{ N}\cdot\text{m}$ ($5-6\text{s}$) | $100\%$ Fixed | $16.423$ | $14.648$ | $33.839$ | $125.66$ | — | — |
| **Scenario F-2** | Adaptive AAN PID | $5.0\text{ N}\cdot\text{m}$ ($5-6\text{s}$) | $20\% - 100\%$ | $16.915$ | $14.999$ | $37.907$ | $\mathbf{124.40}$ | — | — |

### Key Control Findings:
- **Kinematic Tracking:** SMC achieves an **$89.09\%$ reduction in Mean Absolute Error (MAE)** compared to PID ($1.598^\circ$ vs $14.643^\circ$) due to model-based feedforward cancellation of passive joint stiffness and acceleration.
- **Disturbance Rejection:** Under an external $5.0\text{ N}\cdot\text{m}$ spasm torque, SMC suppresses peak error to just **$0.214^\circ$** (compared to $24.599^\circ$ for PID, a **$99.13\%$ suppression**) and settles in **$0.001\text{ s}$** ($1\text{ ms}$).
- **Assist-as-Needed:** Modulating assistance based on tracking error reduces actuator energy while guaranteeing that assistance surges to $100\%$ during disturbance episodes.

---

## 🖼️ Figure Gallery

All figures are automatically generated and saved in the `results/` folder:
- **`fig01_desired_gait_trajectory.png`**: Desired physiological knee joint angle, velocity, and acceleration.
- **`fig02_open_loop_step_response.png`**: Open-loop plant response to $1.0\text{ N}\cdot\text{m}$ torque step ($\zeta = 0.1118$).
- **`fig03_plant_bode_plot.png`**: Open-loop frequency response and resonance peak ($\omega_n = 2.236\text{ rad/s}$).
- **`fig04_pid_step_response.png`**: Closed-loop PID unit-step response ($T_r = 0.119\text{ s}, T_s = 2.35\text{ s}, \text{OS} = 13.8\%$).
- **`fig05_pid_gait_tracking.png`**: Dynamic gait tracking, error, and control torque for baseline PID.
- **`fig06_smc_gait_tracking.png`**: Dynamic gait tracking and equivalent vs switching torque for SMC.
- **`fig07_smc_phase_portrait.png`**: Phase plane trajectory ($e$ vs $\dot{e}$) converging to Hurwitz sliding surface $s = 0$.
- **`fig08_disturbance_profile.png`**: Deterministic $5.0\text{ N}\cdot\text{m}$ human–robot interaction torque pulse ($5.0\text{ s}$ to $6.0\text{ s}$).
- **`fig09_aan_assistance_adaptation.png`**: Multi-panel Assist-as-Needed adaptation showing $\alpha(t)$ surging during disturbance.
- **`fig10_pid_vs_smc_nominal.png`**: Side-by-side nominal tracking comparison between PID and SMC.
- **`fig11_pid_vs_smc_disturbance.png`**: Side-by-side disturbance rejection comparison under $5.0\text{ N}\cdot\text{m}$ load.
- **`fig12_metrics_comparison_barchart.png`**: Grouped statistical performance comparison bar chart.

---

## 📖 Academic Documentation & Viva Defense

- **Detailed Technical Report:** [docs/technical_report.md](docs/technical_report.md)  
  *Contains full 30-section mathematical derivations, physical parameter rationale, Lyapunov stability proofs, clinical context, and extended discussion.*
- **Viva Defense & Demonstration Guide:** [docs/viva_preparation.md](docs/viva_preparation.md)  
  *Contains 35 in-depth viva questions & answers with examiner talking points and a 5-minute evaluation checklist.*

---

## ⚠️ Academic Disclaimer

This project is an undergraduate engineering simulation developed for academic evaluation in BAEEE202 (Control Systems). The plant parameters, trajectory profiles, and disturbance signals are theoretical models. This software is not clinically certified or designed for physical human trials. Real wearable medical robots require multi-channel hardware interlocks, redundant torque sensors, compliance with ISO 13482, and clinical validation.
