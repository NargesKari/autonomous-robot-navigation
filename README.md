

# 📌 Task 1: Localization and Map Handling

## 📖 Theory & Objective

### 🛰️ Map-Based Localization
Localization is the process of estimating the robot's pose $(x, y, \theta)$ relative to a global map. We utilize the **Adaptive Monte Carlo Localization (AMCL)** algorithm, which is a probabilistic approach based on the **Particle Filter**.

* **Particle Filter Mechanism:** AMCL represents the robot's possible positions using a set of "particles." Each particle is a guess of where the robot might be.
* **Prediction Step:** As the robot moves, particles are shifted based on odometry data.
* **Correction Step:** When a Lidar scan is received, particles that match the map's geometry are given higher weights.
* **Resampling:** Over time, particles "converge" or cluster around the most likely position of the robot.



###  TF Tree Structure
A consistent Coordinate Transform (TF) tree is critical for navigation. Our system maintains the following hierarchy:
`map` $\rightarrow$ `odom` $\rightarrow$ `base_link`
![System TF Tree](media/tf_tree.png)

* **Map to Odom:** This transform is published by **AMCL** to correct the "drift" in odometry.
* **Odom to Base Link:** Published by the robot's odometry system (e.g., wheel encoders).

---

## ⚙️ Technical Implementation

### 🗺️ Map Handling & Origin Correction
The occupancy grid map (`depot.pgm`) was processed with a resolution of **0.05 m/pixel**. To align the map's center with the Gazebo world origin $(0,0)$, the `origin` parameter in the YAML file was calculated based on the image dimensions ($604 \times 307$ px):

* **Calculated Origin:** `[-15.1, -7.675, 0.0]`

### 🛠️ Configuration (AMCL & Map Server)
The system parameters are defined in `amcl_config.yaml`. Key configurations include:

| Parameter | Value | Description |
| :--- | :--- | :--- |
| `min_particles` | 500 | Minimum number of particles in the filter |
| `max_particles` | 2000 | Maximum particles for higher uncertainty |
| `update_min_d` | 0.1m | Distance threshold to trigger a filter update |
| `scan_topic` | `/gz_lidar/scan` | Source of Lidar data from Gazebo bridge |
| `odom_model_type` | `diff` | Kinematic model for differential drive |

---

## Execution Workflow

### Launching the Stack
The system is initialized via a single Python launch file:
```bash
ros2 launch robot_description localization.launch.py

```

### 🆕 Testing Methodology (Teleoperation)

To validate the localization performance and observe the particle filter's behavior in real-time, the `teleop_twist_keyboard` node was utilized:

* **Manual Intervention:** By running `ros2 run teleop_twist_keyboard teleop_twist_keyboard`, velocity commands were published to move the robot manually.
* **Validation Strategy:** Since AMCL updates are distance-based (`update_min_d: 0.1m`), manual driving was essential to trigger the filter's update cycle. This allowed the algorithm to compare continuous Lidar scans against the map features, leading to the successful convergence of the particle cloud.

### Demonstration Procedure

1. **Initial Pose Setup:** Used the `2D Pose Estimate` tool in RViz to set an incorrect initial guess, simulating localization recovery.
2. **Convergence Test:** Controlled the robot using `teleop_twist_keyboard`.
3. **Observation:** As the robot moved, the Lidar scans matched against the map features, causing the dispersed particle cloud to converge onto the robot's true pose in both Gazebo and RViz.

---

## 🛠️ Verification and Results
The following video demonstrates:

1. The stability of the **TF Tree**.
2. The robot's ability to recover from an incorrect initial pose estimate.
3. Successful **Particle Cloud Convergence** after a few seconds of movement.
### 🎥 Demonstration Video
![Localization Demo](media/demo1.mp4)


---
