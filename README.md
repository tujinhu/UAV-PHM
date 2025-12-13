------

# 🚁 Digital Twin–Enabled Platform for UAV-PHM

### Rapid Development & Seamless Simulation-to-Reality Migration

This repository provides the **datasets, source code, digital twin models, and unified deployment interfaces** for a **digital-twin-enabled UAV Prognostics and Health Management (UAV-PHM) development and migration framework**.
 Built upon a **high-fidelity RflySim-based UAV digital twin**, the platform enables:

- **credible data generation & consistency enhancement**,
- **cross-scenario algorithm verification**,
- **and seamless Simulation-to-Reality (Sim2Real) migration**.

------

## 📘 Overview

This project establishes a **virtual–physical fusion framework** for UAV-PHM, addressing long-standing bottlenecks such as **real data scarcity**, **low-fidelity simulation**, and **lack of unified algorithm validation pipelines**.
By coupling **real UAVs**, **high-fidelity digital twins**, and a **data consistency enhancement module**, the platform enables **controllable, calibratable, and verifiable** virtual data generation and **cross-platform algorithm deployment**.

### ✨ Key Features

- **Digital-Twin-Driven Credible Data Generation**
   High-fidelity UAV digital twin models built via system identification; consistent multi-condition simulation data aligned with real flight characteristics.
- **Unified Sim2Real Algorithm Migration Interface**
   A standardized communication protocol and model-wrapping mechanism enabling **fast deployment**, **cross-UAV migration**, and **repeatable evaluation**.
- **Simulation-to-Reality Performance Consistency**
   PHM models trained **solely in the digital twin** can achieve **near-real-data performance** when deployed on real UAVs—validated through extensive experiments.
- **Full-Stack Open Platform**
   Includes datasets, DT models, error compensators, mapping pipelines, and validation tools for **reproducible UAV-PHM development**.

------

## 🔧 Installing RflySim

> **Note:** RflySim is a high-fidelity UAV simulation and digital twin toolchain jointly developed by **Prof. Xunhua Dai (Central South University)** and **Feisi Laboratory**.

- GitHub: https://github.com/RflySim
- Official Website: [https://www.rflysim.com](https://www.rflysim.com/)
- Download: https://flyeval.com/docs/#/en/7_DownloadAndSupport/DownloadAndSupport

```bibtex
[1] Dai X, Ke C, Quan Q, et al. RFlySim: Automatic test platform for UAV autopilot systems with FPGA-based hardware-in-the-loop simulations. Aerospace Science and Technology, 2021, 114: 106727.

[2] Dai X, Tu J, Quan Q. RflySim ToolChain: A Rapid Development and Validation Toolchain for Intelligent Unmanned Swarm Systems. Journal of Systems Engineering and Electronics, 2025, 36(4): 1077–1093.
```

------

## 📁 Repository Structure

```
├── data/                               # Real & simulated UAV-PHM datasets
│   ├── Draw/                           # Raw onboard sensor logs
│   ├── Dsim/                           # High-fidelity DT simulation data with SI
│   └── Dpro/                           # Standardized training-ready datasets
│
├── dataSim_without_SI/                 # simulated UAV-PHM datasets without SI
│   ├── Draw/                           # Raw onboard sensor logs
│   ├── Dsim/                           # DT simulation data without SI
│   └── Dpro/                           # Standardized training-ready datasets
│
├── model/
│   ├── dt2real/                        # From COMP to real
│   ├── sim2sim/                        # From WOSI/SI to real
│   ├── real2real/                      # real baseline
│   ├── compare/                        # Universal error compensator 
│
├── verify/                             # Digital twin verification framework suorce code 
│   ├── .../vr2r_ronline/               # Virtual-to-real verification code
│   ├── .../include                     # RflySim SDK
│
└── README.md                           # Documentation
```

------

## 📊 Dataset Description

| Feature         | Description                 |
| --------------- | --------------------------- |
| `gyro_rad[0-2]` | Angular velocity (X/Y/Z)    |
| `acc_m_s2[0-2]` | Linear acceleration         |
| `mag_ga[0-2]`   | Magnetic field measurements |

**Data types:**

- `*raw/` → Real UAV onboard datasets
- `*sim/` → Original DT simulation data
- *`compensated/` → Consistency-calibrated simulation data (post-compensation)

Each dataset is available for PHM tasks including **fault detection, prediction, anomaly estimation, and sim2real transfer evaluation**.

------

## ⚙️ Environment Setup

Dependencies:

```
numpy
scipy
pandas
tqdm
matplotlib
seaborn
scikit-learn
torch==1.13.1
```

------

## 💻 Training & Inference Pipeline

Implemented in:

> ```
> model/dt2real(real2real/sim2sim)/CNN_LSTM_C127(CNN_LSTM_C1247).ipynb
> ```

Includes:

- 📥 **Hybrid data loading** (real + DT-enhanced simulation)
- 🔧 **Consistency compensation** (Universal Error Compensator)
- 🧠 **PHM model training** (classification / regression / uncertainty)

------
