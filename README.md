# Mixed Integer Linear Programming (MILP)

## 🧠 Overview
This project explores **Mixed Integer Linear Programming (MILP)** techniques for **multi-agent formation control**, focusing on coordinating multiple small robots to achieve target spatial configurations.  
The optimization model determines the optimal assignment of agents to formation positions while minimizing total distance and avoiding collisions.  
The system runs on hardware equipped with **NVIDIA integrated GPUs**, enabling real-time optimization and trajectory computation for autonomous formation control.  

This project is part of my semester Project at EPFL in Master 2.



<p align="center">
  <a href="./assets/milp.gif" title="Multi-Agent Formation Control">
    <img src="./assets/milp.gif" width="60%" alt="MILP Multi-Agent Formation Control Demo"/>
  </a>
</p>

---

## ⚙️ Technical Details

| Category | Details |
|-----------|----------|
| **Languages** | Python, C++ |
| **Frameworks / Libraries** | `Gurobi`, `cvxpy`, `NumPy`, `Matplotlib`, `SciPy` |
| **Techniques** | Mixed Integer Linear Programming (MILP), multi-agent optimization, formation assignment, collision avoidance, path planning |
| **Hardware** | Small mobile robots with NVIDIA Jetson / integrated GPUs |
| **Environment** | Ubuntu 22.04, Python 3.10, Gurobi Optimizer or cvxpy backend, VS Code |
| **Features** | Real-time optimization for formation maintenance, flexible MILP formulation for dynamic target patterns, GPU-accelerated computation for fast control updates |


## 📄 Publications

**Semester Project Report** — [Read here](./ressources/Sycamore_semester_project_GABRIEL_PAFFI.pdf)  