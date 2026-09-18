Dead Reckoning ML with FMS CAN & Stratio API

📌 Overview

This repository contains a Machine Learning-based Dead Reckoning (DR) model designed to track continuous vehicle positioning in environments with degraded or denied GPS/GNSS signals.

The system relies on high-frequency vehicle telemetry—specifically FMS CAN bus data—which is extracted and served entirely via the Stratio API. The ML engine processes this data to predict trajectories. To counteract the inherent system drift typical in dead reckoning, the model integrates a validation and correction loop using RF-DETR (Detection Transformer) based locationing.

🏗️ System Architecture

Data Ingestion (Stratio API):

Acts as the centralized data pipeline for the system.

FMS CAN Data: The API provides access to extracted vehicle kinematic data (e.g., wheel speed, steering angle, yaw rate, transmission status, and odometer readings) originally logged from the vehicle's J1939 CAN network.

Initialization: Fetches anchor GPS coordinates (when available) to initialize or anchor the dead reckoning loop.

ML Dead Reckoning Engine:

Processes sequential CAN data fetched from Stratio using [Insert Model Architecture, e.g., LSTM / Transformer / Neural Kalman Filter] to predict continuous relative displacement and heading changes.

Drift Correction (RF-DETR):

Acts as the ground-truth validation layer.

Detects location-specific spatial/RF anchors to compute absolute positioning, resetting the accumulated error (drift) of the DR engine.

⚙️ Prerequisites

Python 3.8+

GPU with CUDA support (recommended for RF-DETR inference and DR model training)

Valid Stratio API credentials (Client ID, Secret, and Fleet/Vehicle IDs)

Installation

Clone the repository and install the required dependencies:

git clone https://github.com/your-org/dead-reckoning-ml.git
cd dead-reckoning-ml
pip install -r requirements.txt
