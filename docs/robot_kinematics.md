# Robot Kinematics Model

## Overview

This document describes the kinematic model used to simulate the motion of the autonomous mobile robot in this project.

The robot is modeled as a **non-holonomic differential drive robot**, meaning it can move forward and rotate but **cannot move sideways**.

This model forms the foundation for the motion simulation, trajectory tracking, and control algorithms used in later stages of the project.

------------------------------------------------------------------------------------------------------------------------------

# Robot State Representation

The robot pose in the global coordinate frame is defined as:

X = [x, y, θ]

Where:

- **x** → Robot position along the X-axis (meters)
- **y** → Robot position along the Y-axis (meters)
- **θ** → Robot orientation (heading angle) measured from the positive X-axis (radians)

This representation describes the **pose of the robot in a 2D plane**.

---

# Control Inputs

The robot is controlled using two inputs:

u = [v, ω]

Where:

- **v** → Linear velocity (m/s)
- **ω** → Angular velocity (rad/s)

Interpretation:

- Linear velocity moves the robot **forward along its heading direction**
- Angular velocity rotates the robot **about its vertical axis**

Positive angular velocity corresponds to **counterclockwise rotation (CCW)**.

---

# Robot Motion Model

The robot motion is governed by the following kinematic equations:

x_dot = v cos(θ)

y_dot = v sin(θ)

θ_dot = ω

These equations describe how the robot's position and orientation change over time.

### Physical Meaning

- The robot moves forward in the direction it is facing.
- The heading angle determines the direction of motion.
- Angular velocity controls how quickly the robot rotates.

This model assumes that the robot behaves as a **non-holonomic system**, meaning it cannot move laterally.

---

# Discrete Time Simulation

Since computers simulate motion in discrete time steps, the continuous equations are converted into discrete updates.

Using a time step **dt**, the robot state is updated as:

x_next = x + v cos(θ) dt

y_next = y + v sin(θ) dt

θ_next = θ + ω dt

This update rule is implemented in the function:

------------------------------------------------------------------------------------------------------------------------------

# robot_step.m


which computes the next robot state based on the current state and control input.

---

# Coordinate System

The simulation assumes a standard 2D coordinate frame:

- X-axis points to the right
- Y-axis points upward
- θ is measured from the positive X-axis

Robot motion is always aligned with its heading direction.

---

# Assumptions

The kinematic model makes the following simplifying assumptions:

- No wheel slip
- Perfect velocity control
- Flat ground surface
- No sensor noise
- Instantaneous response to control inputs

These assumptions allow us to focus on the robot's motion behavior before introducing more complex dynamics.

---

# Validation Tests (test_robot_motion.m)

To verify the correctness of the robot model, the following tests are performed.

### Test 1: Straight Line Motion

Inputs:

v > 0  
ω = 0  

Expected behavior:

Robot moves in a straight line along its current heading.

---

### Test 2: Pure Rotation

Inputs:

v = 0  
ω ≠ 0  

Expected behavior:

Robot rotates in place while its position remains unchanged.

---

### Test 3: Circular Motion

Inputs:

v > 0  
ω ≠ 0  

Expected behavior:

Robot follows a circular trajectory.

The radius of the circle is given by:

R = v / ω

---

# Role in the Robotics System

The robot kinematic model is the **core motion model** used throughout the project.

It is used by:

- Motion simulation
- Path tracking controllers
- Trajectory generation
- State estimation algorithms

Future project components such as **PID control, LQR control, and Kalman filtering** will rely on this model.

---

# Related Implementation Files

The robot motion model is implemented in:

matlab/simulation/robot_step.m

Basic simulation is implemented in:

matlab/simulation/basic_robot_motion.m

Model validation tests are located in:

matlab/simulation/test_robot_motion.m

------------------------------------------------------------------------------------------------------------------------------
