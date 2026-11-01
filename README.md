# 3D Printer Modelling Mini Project 2026

**TU Delft MSc Systems & Control**  
**Course:** Modelling of Dynamical Systems

## 1. Project Overview

The objective of this project is to develop a mathematical model and simulation of a CoreXY 3D printer, integrating continuous physical dynamics, discrete control logic, and hybrid system behaviour.

The project is divided into three main parts:

1. **Physical Modelling:** Derivation of the kinematic and dynamic equations governing the printer head, including the electrical and mechanical behaviour of the driving motors. The thermal dynamics of the nozzle and the cooling of deposited filament are also modelled.

2. **Finite State Machines and Automata:** Development of discrete models describing the filament replacement procedure and the detection of the printer head's parked position.

3. **Hybrid System Modelling:** Integration of the continuous physical models and discrete control logic into a unified hybrid system capable of simulating printing operations, filament replacement, and trajectory execution.

## 2. Modelling Assumptions

To obtain a manageable mathematical model, the following assumptions were adopted:

1. **Planar motion:** The printer head moves only in the x-y plane, neglecting motion along the z-axis.
2. **Ideal belt transmission:** The belts are assumed massless, inextensible, and free of slipping, while the mechanical components are considered rigid.
3. **Motor behaviour:** Both motors are assumed identical and are modelled as DC motors with linear electrical and mechanical characteristics. Internal motor friction is neglected, while bearing friction is considered.
4. **Bearing friction:** Friction associated with the linear bearings and rolling components is approximated using linear viscous damping.
5. **Thermal behaviour:** The nozzle is treated as a lumped thermal system with a spatially uniform temperature, while heat losses to the environment and filament are represented by an equivalent thermal resistance.
6. **Filament cooling:** The deposited filament is modelled as a one-dimensional distributed-parameter system with a uniform square cross-section, considering heat exchange with the heated bed and surrounding air as the computational domain grows during printing.
7. **Discrete operations:** Control events and transitions between operating modes are assumed instantaneous, while the physical evolution between events is governed by the corresponding continuous dynamics.
8. **Initial position:** The print head initially starts at the origin (x₀, y₀) = (0, 0), with both motor angles equal to zero.
9. 
Additional assumptions and parameter choices are introduced and justified where required in the individual models.

## 3. Repository Structure

The repository is organised into two main folders:

- **`matlab-code/`** contains the MATLAB scripts used for each problem, including the models and simulations.
- **`matlab-figures/`** contains the corresponding figures and simulation results.

Within each folder, files are organised by problem.

## 4. Software

The numerical simulations are developed in **MATLAB**, using numerical methods such as `ode15s`, finite differences, and the Forward Euler method, depending on the model being solved.

## 5. Repository History and Access

This repository was created at the beginning of the project.

As this was our first experience with GitHub, we were initially unfamiliar with its version-control features and developed most of our work locally. We therefore primarily used the repository as an organised location for storing and submitting our MATLAB code and results, meaning that its commit history does not reflect the complete development process.

The repository is publicly accessible through the GitHub link provided in the report's Appendix and in the final Brightspace submission.
