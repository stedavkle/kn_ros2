Of course. Here is a comprehensive summary of the complete ROS2 project architecture, incorporating all discussed components. This document is designed to serve as a detailed blueprint for the development team.

---

### **ROS2 System Architecture for Nanoprober Automation**

**Version 1.0**

**Objective:** This document outlines the complete software architecture for the ROS2-based control system for the nanoprobing platform. The design encapsulates hardware control, computer vision, simulation, and path planning into distinct, modular components to ensure a robust, scalable, and maintainable system.

#### **Core Architectural Principles:**

*   **Node per Responsibility:** Each major function (hardware control, vision, simulation, planning) is handled by a dedicated ROS2 node.
*   **Centralized Orchestration:** A `nanoprober_coordinator` node acts as the "brain," containing the high-level application logic and commanding the other nodes.
*   **Data-Centric Communication:** Nodes communicate via well-defined ROS2 topics, services, and actions using standardized and custom message types.
*   **Explicit State Management:** The state of the robotic system is managed transactionally by the coordinator to ensure causality and prevent race conditions, while non-critical state is updated reactively.

---

### **1. ROS2 Project Structure**

A single ROS2 package, `nanoprober_bringup`, will contain all necessary code.

```
ros2_ws/
└── src/
    └── nanoprober_bringup/
        ├── package.xml               # Defines dependencies (rclpy, vision_msgs, etc.)
        ├── setup.py                  # Defines executable entry points for each node
        ├── nanoprober_bringup/
        │   ├── nodes/
        │   │   ├── manipulator_driver_node.py
        │   │   ├── sem_driver_node.py
        │   │   ├── vision_node.py
        │   │   ├── simulation_node.py
        │   │   ├── path_planning_node.py
        │   │   └── nanoprober_coordinator_node.py
        │   ├── hal/                  # Low-level hardware wrappers (nanocontrol.py)
        │   ├── controller/           # Mid-level control logic (manipulator.py, encoder.py)
        │   ├── microscope/           # SEM-specific logic (SEM_API.py, sem.py)
        │   ├── vision/               # AI vision logic (detector.py, tracking/core.py)
        │   ├── planning/             # Path planning algorithms (e.g., ICBS implementation)
        │   └── simulation/           # 3D scene and collision checking logic
        ├── launch/
        │   └── nanoprober.launch.py  # Main launch file to start and configure all nodes
        ├── config/
        │   └── system_config.yaml    # All system parameters (comports, tip geometries, calibration)
        ├── msg/
        │   ├── SEMScan.msg           # Custom message for SEM image + metadata
        │   └── TipDetections.msg     # Custom message for vision results
        ├── srv/
        │   ├── UpdateManipulatorState.srv
        │   └── CheckXYCollision.srv
        └── action/
            └── PlanMultiPath.action
```

---

### **2. Node Implementation Details**

#### **A. `manipulator_driver`**
*   **Purpose:** Low-level control for a *single* manipulator. One instance launched per device.
*   **Core Logic:** Wraps `NanoControl` and `ManipulatorEncoded` classes.
*   **ROS2 Interfaces (Namespaced, e.g., `/manipulator1/...`):**
| Type | Name | Message Type | Role | Description |
|---|---|---|---|---|
| **Action** | `move_relative` | `your_action/MoveRelative` | Server | Executes a relative move in 3D metric coordinates. |

---
#### **B. `sem_driver`**
*   **Purpose:** Sole interface to the Scanning Electron Microscope (SEM).
*   **Core Logic:** Wraps the `ZEISSSem` class.
*   **Platform Constraint:** **Must run on Windows** with Zeiss SmartSEM API.
*   **ROS2 Interfaces:**
| Type | Name | Message Type | Role | Description |
|---|---|---|---|---|
| **Topic** | `/sem/scan` | `SEMScan.msg` | Publisher | Publishes the complete SEM image and metadata after a scan. |
| **Topic** | `/sem/status` | `your_msg/SEMStatus`| Publisher | Publishes asynchronous changes in SEM state (e.g., magnification, WD). |
| **Action**| `grab_image`| `your_action/GrabImage`| Server | Commands a full scan; returns the `SEMScan` message as the result. |
| **Action**| `auto_focus`| `your_action/AutoFocus`| Server | Performs the auto-focus routine. |
| **Service**| `set_magnification` | `std_srvs/SetInt` | Server | Sets the SEM magnification. |

---
#### **C. `vision_node`**
*   **Purpose:** Performs all AI vision tasks (detection, pose estimation, tracking).
*   **Core Logic:** Wraps `Detector` and tracker classes.
*   **Platform Constraint:** Recommended to run on a machine with a **dedicated GPU**.
*   **ROS2 Interfaces:**
| Type | Name | Message Type | Role | Description |
|---|---|---|---|---|
| **Topic** | `/sem/scan` | `SEMScan.msg` | Subscriber | Primary input for image processing. |
| **Topic** | `/vision/tip_detections` | `TipDetections.msg`| Publisher | Publishes results of detection/tracking for each processed frame. |
| **Topic** | `/vision/annotated_image`| `sensor_msgs/Image`| Publisher | Debug image with annotations for visualization. |

---
#### **D. `simulation_node` (The Digital Twin)**
*   **Purpose:** Maintains a virtual state of the workspace for "what-if" collision checking.
*   **Core Logic:** Manages a 3D scene; uses a collision-checking library.
*   **ROS2 Interfaces:**
| Type | Name | Message Type | Role | Description |
|---|---|---|---|---|
| **Topic** | `/sem/status` | `your_msg/SEMStatus` | Subscriber | Reactively updates its internal SEM state (e.g., magnification). |
| **Service** | `/simulation/update_manipulator_state`| `UpdateManipulatorState.srv`| **Server** | **Critical Interface.** Explicitly updates manipulator XYZ states. Called by the coordinator *after* a verified move. |
| **Service** | `/simulation/check_xy_collision` | `CheckXYCollision.srv` | **Server** | Takes future XY positions for all tips and returns `bool approved`. Used by the path planner. |
| **Service**| `/simulation/check_z_collision`| `your_srv/CheckZCollision`| **Server** | Takes a future Z position for the sample stage and checks for collisions. |

---
#### **E. `path_planning_node`**
*   **Purpose:** Generates collision-free, time-coordinated paths for multiple manipulators.
*   **Core Logic:** Implements the Improved Conflict-Based Search (ICBS) algorithm.
*   **ROS2 Interfaces:**
| Type | Name | Message Type | Role | Description |
|---|---|---|---|---|
| **Action** | `/planning/plan_paths` | `PlanMultiPath.action` | **Server** | **Primary Interface.** Takes start/goal pairs; result is a list of waypoint paths. |
| **Service** | `/simulation/check_xy_collision` | `CheckXYCollision.srv` | **Client** | **Crucial Dependency.** Repeatedly calls the simulation node to validate states during its search. |

---
#### **F. `nanoprober_coordinator` (The Brain)**
*   **Purpose:** Contains high-level application logic; orchestrates all other nodes.
*   **Core Logic:** Implements state machines and control loops. Performs all coordinate transformations between pixel space and manipulator space.
*   **ROS2 Interfaces:** Primarily a client to other nodes.
| Type | Name | Message Type | Role | Description |
|---|---|---|---|---|
| **Topic** | `/vision/tip_detections`| `TipDetections.msg`| Subscriber | Receives verified tip positions to close the control loop. |
| **Topic** | `/sem/scan` | `SEMScan.msg` | Subscriber | Receives image metadata for coordinate transformations. |
| **Action** | `move_relative` | `your_action/MoveRelative` | Client | Sends goals to `manipulator_driver` nodes. |
| **Action**| `grab_image` | `your_action/GrabImage` | Client | Commands the `sem_driver` to acquire images. |
| **Action** | `/planning/plan_paths` | `PlanMultiPath.action` | Client | Requests a safe, multi-agent plan before execution. |
| **Service**| `/simulation/update_manipulator_state`| `UpdateManipulatorState.srv` | Client | Keeps the digital twin synchronized with reality after every verified move. |

---

### **4. System-Wide Workflow Example: Coordinated Multi-Tip Move**

This workflow demonstrates the collaboration between all nodes for a complex task.

1.  **Goal:** The `coordinator` needs to move manipulators #2 and #4 to new goal positions simultaneously.
2.  **State Sync:** The `coordinator` calls `/simulation/update_manipulator_state` with the latest verified system state to ensure the digital twin is accurate.
3.  **Plan Request:** The `coordinator` sends a goal to the `/planning/plan_paths` action server, providing the start and goal positions for manipulators #2 and #4.
4.  **Planning Phase:** The `path_planning_node` starts its ICBS search. During the search, it repeatedly calls the `/simulation/check_xy_collision` service to validate potential moves and build a conflict-free plan.
5.  **Plan Reception:** The `path_planning_node` completes and returns the final waypoint paths to the `coordinator`.
6.  **Execution Phase:** The `coordinator` begins a loop to execute the paths waypoint-by-waypoint:
    a.  For the current waypoint, it performs the **coordinate transformation** from pixel space to 3D manipulator commands for both tips.
    b.  It sends `move_relative` action goals to **both** `/manipulator2/` and `/manipulator4/` in parallel and waits for both to complete.
    c.  It calls the `/sem/grab_image` action. The `sem_driver` publishes a `SEMScan`, which the `vision_node` processes and publishes a `TipDetections` message.
    d.  The `coordinator` receives the verified positions and confirms the step was successful.
    e.  It calls `/simulation/update_manipulator_state` to keep the digital twin in sync.
    f.  The loop continues until all waypoints are executed.