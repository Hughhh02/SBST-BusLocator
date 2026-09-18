# Vision-Kinematic Ego-Localization for GPS-Denied Transit Depots
This repository contains the software pipeline for an onboard Indoor Positioning System (IPS) designed for SBS Transit (SBST) buses operating in GPS-denied environments (e.g., underground depots).

The system provides continuous, zero-drift localization by fusing real-time visual anchors (RF-DETR) with batched cloud-based engine telemetry (Stratio API) using a dynamically calibrating, Delay-State Extended Kalman Filter (EKF).


📌 System Architecture
The localization pipeline completely eliminates the need for expensive infrastructure (e.g., LiDAR, BLE beacons, UWB) by treating the bus as an autonomous agent. It operates across two parallel engines:

**1. Kinematic Engine (Dead Reckoning)**
Because direct J1939 CAN tapping and steering angle sensors are unavailable, the system relies on differential kinematics pulled from the Stratio Predictive Maintenance API.

Longitudinal Speed: Derived from Vehicle Speed (ID 114) or Tacho Speed (ID 460).

Yaw Rate (Heading): Calculated via differential wheel speeds using the Front Left (ID 463) and Front Right (ID 464) axle rotations divided by the physical track width of the bus chassis.

Function: Pushes the bus state forward continuously on a 2.5D Topological Graph.

**2. Vision Engine (Absolute Anchoring)**
An onboard CCTV camera runs an edge-optimized RF-DETR (Detection Transformer) model to provide absolute spatial ground truths.

Landmark Detection: Detects known static depot infrastructure (e.g., numbered berths, painted pillars).

Coordinate Lookup: Translates the 2D bounding box into a physical distance and queries a lightweight JSON/CSV 2D Lookup Table (LUT).

Function: Outputs absolute [X, Y, Z-floor] coordinates to instantly kill dead reckoning drift.

**3. Sensor Fusion (Delay-State EKF)**
Fusing real-time edge video with batched cloud telemetry introduces significant asynchronous latency. The system uses a Delay-State EKF to solve this:

Retrospective Updating: Maintains a timestamped rolling buffer of kinematic states. When a delayed visual anchor arrives, the EKF rewinds the buffer to the exact UNIX timestamp of the video frame, applies the absolute coordinate, and fast-forwards the math back to the present millisecond.

Dynamic Bias Calibration: The EKF tracks a fourth state (Scale_Factor) to dynamically learn and correct systematic kinematic errors (e.g., worn tire treads, varying passenger weight) based on observed visual discrepancies.

🚀 Key Features
Hardware-Light: Requires only a standard onboard IP camera and an active Stratio API token.

Self-Calibrating Odometry: The system learns and filters kinematic bias over time using visual ground truths.

2.5D Topological Mapping: Abandons heavy 3D CAD/BIM models in favor of a lightweight node-edge map. Ramp elevations are handled logically via state-machine transitions rather than barometric sensors.

Out-of-Sequence Measurement (OOSM) Handling: Safely fuses high-frequency local video with low-frequency batched cellular API payloads without mathematical divergence.
