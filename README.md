# Urban-Driving Optimal Planning for HVs

![License](https://img.shields.io/badge/license-Academic-blue)
![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![Gurobi](https://img.shields.io/badge/Gurobi-required-red)
![SUMO](https://img.shields.io/badge/SUMO-visualization-green)

> Codes for **urban-driving optimal planning for hybrid vehicles** with integrated lane-change, intersection, safety, and powertrain constraints.

---

## Overview

This repository contains the simulation codes, scenario-generation scripts, optimization models, baseline-comparison tools, plotting scripts, and SUMO visualization pipeline for the manuscript:

**Urban-Driving Optimal Planning for HVs with Integrated Lane-Change, Intersection, Safety, and Powertrain Constraints**

The framework solves finite-horizon eco-safe driving problems for a hybrid vehicle in rule-rich urban environments. It jointly optimizes lane selection, lane changes, longitudinal motion, safety constraints, signalized and unsignalized intersection behavior, traction and braking forces, engine power, motor power, battery power, state of charge, fuel consumption, and equivalent energy.

---

## Main Capabilities

### Full Hybrid Vehicle Planner

The proposed planner solves a mixed-integer nonconvex optimization problem that integrates:

- lane occupancy and lane-change logic,
- safe car-following and lane-change gap constraints,
- signalized intersection constraints,
- unsignalized stop/yield constraints,
- longitudinal vehicle dynamics,
- hybrid powertrain dynamics,
- fuel, battery, SOC, and equivalent-energy terms.

### Scenario Generation

The repository includes synthetic urban driving scenarios with:

- multi-lane roads,
- lane-dependent speed limits,
- road curvature and grade,
- surrounding vehicles,
- lane closures,
- emergency lane-change events,
- signalized intersections,
- unsignalized intersections.

### Baseline Comparison

The benchmark tools compare the proposed full planner with:

- RF: rule following,
- HCS-i NLC: signal-aware no-lane-change baseline,
- OEAC-i: overtaking-enabled signalized eco-approach baseline,
- KINO: kinematic only optimization,
- Full: proposed hybrid vehicle planner.

The benchmark reports success rate, distance, target travel time, lane changes, equivalent energy, equivalent energy per kilometer, comfort metrics, solver/controller time, and adjusted metrics for failed cases.

### SUMO Visualization

Optimized trajectories can be exported to SUMO for visual inspection, frame generation, and video creation.


### Citation

If you use this software in your research, please cite:

> M. Hasanzadeh and A. Kargarian, “Urban-Driving Optimal Planning for HVs with Integrated Lane-Change, Intersection, Safety, and Powertrain Constraints.”

---

### Authors

**Milad Hasanzadeh**
Department of Electrical and Computer Engineering
Louisiana State University
Email: [mhasa42@lsu.edu](mailto:mhasa42@lsu.edu)

**Amin Kargarian**
Department of Electrical and Computer Engineering
Louisiana State University
Email: [kargarian@lsu.edu](mailto:kargarian@lsu.edu)

---

### License

Academic and research use only.

---

### Release Information

Release date: July 2026

