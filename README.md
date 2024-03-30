# Longitudinal_Control
![architecture](https://github.com/Jay-Wang77/Longitudinal_Control/assets/137873786/cfe8b027-14c6-4522-beb7-1a88f1ed7617)

A force balance along the vehicle longitudinal axis yields:

## Longitudinal force
m𝑎 = F𝑥𝑓 + F𝑥𝑟 - F𝑎 - R𝑥𝑓 - R𝑥𝑟 - mgsin(θ)
![image](https://github.com/Jay-Wang77/Longitudinal_Control/assets/137873786/a6b5a0b3-482f-406c-b4dc-d616eed46f72)

where:
- F𝑥𝑓 is the longitudinal tire force at the front tires,
- F𝑥𝑟 is the longitudinal tire force at the rear tires,
- F𝑎 is the equivalent longitudinal aerodynamic drag force,
- R𝑥𝑓 is the force due to rolling resistance at the front tires,
- R𝑥𝑟 is the force due to rolling resistance at the rear tires,
- m is the mass of the vehicle,
- g is the acceleration due to gravity,
- θ is the angle of inclination of the road on which the vehicle is traveling.

## Vehicle Longitudinal control
"Longitudinal controller" is typically used in referring to any control system that controls the longitudinal motion of the vehicle.

**Control state**: longitudinal velocity, acceleration or its longitudinal distance

**Control input**: (the actuators used to implement longitudinal control): throttle and brakes

## PID Controller
![image](https://github.com/Jay-Wang77/Longitudinal_Control/assets/137873786/0626f643-bb7e-4079-a0b2-b3610fe115c9)


### Proportional (P) Control:
**Definition**: Multiplying the error signal by a constant factor.

**Purpose**: It speeds up the signal's arrival at the target and enlarges the error signal (difference between desired speed and actual speed) by multiplying it with a proportional term (P), thereby accelerating towards the desired speed and reducing steady-state error.

**Issues**:
1. Steady-state error always exists.
2. When the P value is small, response speed is slow, and the final error from the target is large; when the P value is large, response speed is fast, but oscillations are large.


### Integral (I) Control:
**Definition**: Integrating (lagging) the signal.

**Purpose**: Eliminates steady-state error, no differential adjustment.

**Issues**: May cause oscillations. For example, if the target is 10 and it decreases from 50 to 10, but drops to 0 before rising back to 10 (although convergence is achieved, overshoot occurs), ideal control should gradually decrease from 50 to 10 without overshoot.

### Derivative (D) Control:
**Definition**: Differentiating (anticipating) the signal.

**Purpose**: Suppresses oscillations, no differential adjustment.

**Issues**: Unable to eliminate static residual error.

### Summary:
1. The proportional term (P) can speed up the signal's arrival at the target. When P is too large, it can cause overshoot in practical applications (due to delay), which can be compensated by using derivative (D) to eliminate overshoot caused by the proportional term (P) in practical applications. Integral (I) term is generally not commonly used (reducing overshoot from derivative D is relatively easy, but reducing overshoot from integral I is difficult). Although the integral term (I) can eliminate steady-state error, it may introduce significant overshoot, and its elimination is cumbersome (unless the steady-state error is particularly large). Generally, the overshoot introduced by the integral term can only be reduced by lowering the proportional term (slightly reducing P), but reducing the proportional term (P) leads to slower response. In general control, proportional term (P) and derivative term (D) are sufficient to ensure error within a certain tolerance range.

2. In practice, the empirical approach is to adjust the proportional term (P) first, then the derivative term (D), and finally the integral term (I). As long as the control accuracy reaches a certain range, it is sufficient, without requiring complete precision. Achieving very high precision requires increasing the integral term (I) to eliminate steady-state error, which inevitably leads to overshoot, and then increasing the derivative term (D) to suppress overshoot, which inevitably increases noise (causing the log amplitude-frequency curve of the Bode plot to tilt upwards).

3. P + D: Fast and meets accuracy requirements, I: Provides a very small integral term (I) to reduce steady-state error.

## Getting Started

### Requirements
- Ubuntu 20.04
- ROS foxy
- Additional installation of carla-ros-bridge packages and other packages is required.
- Carla 0.9.13

1. **Enter Workspace Folder**
    Navigate to the workspace folder where you want to build the project.

    ```bash
    cd your_workspace_path
    ```

2. **Build the Project**
    Use `catkin_make` to build the ROS packages.

    ```bash
    colcon build
    ```

3. **Source the Setup Script**
    To configure the environment variables for the project, source the setup script.

    ```bash
    source ./install/setup.bash
    ```
4. **Run Carla simulator**
     Navigate to the Carla simulator directory and run the following command.

    ```bash
    ./CarlaUE4.sh
    ```
5. **Launching the Project**
     Navigate to the ROS workspace (carla-ros-bridge) and follow these steps in order:

    a. Launch the Carla ROS bridge ego vehicle visualization:
    ```bash
    ros2 launch carla_bridge_ego_vis carla_bridge_ego_vehicle.launch.py
    ```

    b. Run the Carla PID controller node:

    ```bash
    ros2 run carla_pid_controller carla_shenlan_pid_controller_node
    ```
