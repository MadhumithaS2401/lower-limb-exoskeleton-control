# Lower-Limb Exoskeleton for Rehabilitation (Control Systems Simulation)

**Course Code:** BAEEE202 – Control Systems (Semester 3, B.Tech ECE)  
**Authors:** Madhumitha (25BEL0042) & Monica Shree (25BEL0039)  

---

## 📌 Project Overview
This repository contains a MATLAB & Simulink simulation project for an active **Lower-Limb Rehabilitation Exoskeleton**. The system controls hip and knee sagittal motion to restore physiological walking patterns in patients with motor impairment, featuring:
- Biomechanically accurate human gait trajectory generation.
- 2-DOF nonlinear rigid body dynamics (Euler-Lagrange).
- Baseline Classical Control (Tuned PID with Gravity Pre-Compensation).
- Advanced Robust Control (Sliding Mode Control - SMC).
- Assist-as-Needed (AAN) adaptive strategy with human-robot interaction disturbance modeling.
- Comprehensive performance evaluation (RMSE, MAE, ITAE, Control Energy).

---

## 📁 Repository Structure
```text
Lower-Limb-Exoskeleton-Rehabilitation/
├── config/        # System physical & simulation parameters
├── trajectories/  # Biomechanical gait trajectory generator
├── models/        # 2-DOF dynamic plant & disturbance models
├── controllers/   # PID, Sliding Mode Controller & AAN algorithms
├── scripts/       # Testbenches & automated batch simulation runners
├── results/       # Simulation figures and performance metric CSVs
└── docs/          # Theoretical derivations and viva defense guide
```
