# Development of a Simulation Environment for an Unmanned Surface Vehicle (USV)

This project focuses on the development of a **simulation environment for an Unmanned Surface Vehicle (USV)** using **Gazebo Sim, ROS 2, and the Virtual RobotX (VRX) framework**.

The main objective is to provide a realistic simulation environment for the **WAM-V surface vehicle**, study its dynamic behavior, and develop an autonomous navigation system based on a **proportional guidance law**.

---

## Project Overview

Unmanned Surface Vehicles (USVs) are increasingly used for applications such as marine monitoring, environmental inspection, search and rescue, and marine pollution detection.

Developing navigation algorithms directly on a real vehicle can be expensive and difficult. Simulation provides a controlled environment in which the vehicle dynamics, environmental disturbances, sensors, and navigation algorithms can be studied before deployment on a real platform.

This project therefore develops a simulation environment based on the **WAM-V USV available in VRX**.

The work includes:

- Configuration of the WAM-V model
- Simulation of marine vehicle dynamics
- Study of the 3-DOF mathematical model
- Integration of environmental disturbances
- Sensor simulation
- Development of a proportional guidance algorithm
- Autonomous waypoint/target navigation
- Evaluation of the vehicle trajectory and guidance performance

---

## Objectives

The main objectives of the project are:

1. Develop a simulation environment for a WAM-V USV.
2. Configure the vehicle in Gazebo Sim using the VRX framework.
3. Study the mathematical and hydrodynamic model of the vehicle.
4. Analyze the behavior of the WAM-V under different environmental conditions.
5. Implement a proportional guidance law for autonomous navigation.
6. Control the vehicle using the left and right thrusters.
7. Evaluate the ability of the vehicle to follow a desired trajectory.
8. Provide a foundation for future USV autonomy and marine waste detection research.

---

## Technologies

| Component | Technology |
|---|---|
| Operating System | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |
| Simulator | Gazebo Sim |
| USV Framework | VRX |
| Vehicle | WAM-V |
| Programming | Python / C++ |
| Robotics | ROS 2 |
| Guidance | Proportional Guidance |
| Visualization | Gazebo Sim |

---

# Simulation Environment

The simulation is based on the **WAM-V** vehicle provided by the Virtual RobotX (VRX) framework.

VRX provides a simulation environment for the development and evaluation of autonomous surface vehicle systems.

The WAM-V operates on the water surface and is controlled through its propulsion system.

The simulation allows the vehicle to interact with:

- Water dynamics
- Wind
- Waves
- Hydrodynamic forces
- Vehicle inertia
- Thruster forces

---

# Mathematical Model

For navigation purposes, the vehicle is modeled using a simplified **3-degree-of-freedom (3-DOF)** model.

The considered degrees of freedom are:

- **Surge** — longitudinal motion
- **Sway** — lateral motion
- **Yaw** — rotation around the vertical axis

The vehicle position and orientation can be represented by:

```text
η = [x, y, ψ]ᵀ
```

where:

- `x` is the longitudinal position
- `y` is the lateral position
- `ψ` is the yaw angle

The body-fixed velocities are:

```text
ν = [u, v, r]ᵀ
```

where:

- `u` is the surge velocity
- `v` is the sway velocity
- `r` is the yaw rate

The general marine vehicle dynamic model can be expressed as:

```text
Mν̇ + C(ν)ν + D(ν)ν = τ + τenvironment
```

where:

- `M` represents the inertia and added-mass effects
- `C(ν)` represents Coriolis and centripetal effects
- `D(ν)` represents hydrodynamic damping
- `τ` represents the control forces and moments
- `τenvironment` represents environmental disturbances

The simulation uses the existing VRX/Gazebo vehicle dynamics rather than recreating the complete vessel dynamics from scratch.

---

# Environmental Conditions

An important part of the project is the study of the WAM-V under environmental disturbances.

The simulation can include:

- Wind
- Waves
- Hydrodynamic effects
- Vehicle inertia
- Hydrodynamic damping

Different environmental conditions can be used to evaluate the robustness of the navigation and guidance system.

For example, the vehicle can be tested under different wind velocities and wave conditions while following a desired trajectory.

---

# Proportional Guidance

The autonomous navigation system is based on a **proportional guidance law**.

The objective of the guidance algorithm is to determine the desired heading of the WAM-V according to its position relative to the target.

The navigation system continuously calculates the relative position between the vehicle and the target:

```text
Δx = xgoal - x
Δy = ygoal - y
```

The desired heading can then be determined from the direction toward the target:

```text
ψdesired = atan2(Δy, Δx)
```

The heading error is calculated as:

```text
eψ = ψdesired - ψ
```

The angular command is then determined using a proportional relationship:

```text
rcommand = Kp · eψ
```

where:

- `Kp` is the proportional gain
- `eψ` is the heading error
- `rcommand` is the desired yaw-rate command

The heading error is normalized to ensure that the vehicle takes the appropriate rotational direction.


---

# References

### Virtual RobotX (VRX)

This project uses the Virtual RobotX framework for the WAM-V simulation environment.

VRX provides simulation tools and environments for the development and evaluation of autonomous surface vessel systems.

### Fossen

Fossen, T. I., *Handbook of Marine Craft Hydrodynamics and Motion Control*, Wiley.

The theoretical framework provides the basis for modeling and analyzing marine vehicle dynamics and motion control.

---

# Author

**Oussama Ismaili** and **Fnadi Mohamed**

Engineering Student — Computer Science, Data Science & Artificial Intelligence

GitHub: [oussamaissm](https://github.com/oussamaissm)

---

## License

This project contains components based on open-source frameworks and libraries. Please refer to the respective licenses of the included third-party software, including VRX, Gazebo Sim, and ROS 2.
