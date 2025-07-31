# Robot-Operating-System
# ROS2 Hands-on Practice for Robotics

This repository contains **10 practical ROS2 projects** designed to help understand and master the **core concepts of ROS2**.  
The tasks gradually cover topics, services, actions, parameters, lifecycle management, TF2, plugin architecture, and a simplified Navigation (Nav2) flow.  
Completing these exercises will prepare you for working with **ROS2 core libraries** like `rclcpp`, and complex frameworks like **Navigation2 (Nav2)**.

---

## **Repository Structure**
.
├── Task01_PubSub
├── Task02_CustomMsg
├── Task03_Service
├── Task04_Action
├── Task05_Parameters
├── Task06_LifecycleNode
├── Task07_Pluginlib
├── Task08_TF2
├── Task09_Composition
└── Task10_MiniNavDemo


Each folder contains:
- `src/` → ROS2 node source files  
- `msg/`, `srv/`, or `action/` → Custom interfaces (when applicable)  
- `CMakeLists.txt` & `package.xml` → ROS2 package configuration  
- `README.md` → Task-specific details and run instructions  

---

## **Tasks Overview**

### **Task 1: Minimal Publisher & Subscriber**
- **Goal:** Implement a basic publisher and subscriber node.
- **Key Concepts:** `rclcpp::Node`, `create_publisher`, `create_subscription`.
- **Deliverable:** Publishes random integers, subscriber prints them.

---

### **Task 2: Custom Message & Interface**
- **Goal:** Learn how to create and use custom message types.
- **Key Concepts:** `rosidl`, custom `.msg` files, message generation.
- **Deliverable:** Publishes custom `SensorReading` messages and subscribes to them.

---

### **Task 3: ROS2 Service**
- **Goal:** Implement request-response communication.
- **Key Concepts:** `create_service`, `create_client`, synchronous and asynchronous calls.
- **Deliverable:** Simple `AddTwoInts` service and client.

---

### **Task 4: ROS2 Action**
- **Goal:** Handle long-running tasks with progress feedback.
- **Key Concepts:** Action servers, action clients, feedback and result handling.
- **Deliverable:** `MoveRobot.action` with progress updates.

---

### **Task 5: Parameters & Dynamic Reconfigure**
- **Goal:** Tune node parameters at runtime.
- **Key Concepts:** `declare_parameter`, `get_parameter`, parameter callbacks.
- **Deliverable:** Node with dynamically adjustable speed and name parameters.

---

### **Task 6: Lifecycle Node**
- **Goal:** Understand deterministic node state transitions (important for Nav2).
- **Key Concepts:** Managed nodes, transition services.
- **Deliverable:** Lifecycle node with `configure()`, `activate()`, `deactivate()`.

---

### **Task 7: Pluginlib**
- **Goal:** Implement a runtime plugin architecture.
- **Key Concepts:** Abstract base classes, `pluginlib` registration, dynamic loading.
- **Deliverable:** Two controller plugins (`SimpleController`, `AdvancedController`) loaded dynamically.

---

### **Task 8: TF2 & Robot State Broadcasting**
- **Goal:** Work with robot frames and coordinate transforms.
- **Key Concepts:** `tf2_ros::TransformBroadcaster`, `tf2_ros::Buffer`, `lookupTransform`.
- **Deliverable:** Broadcast a moving transform (`odom → base_link`) and query it from another node.

---

### **Task 9: Composition (Nodelets Style)**
- **Goal:** Launch multiple nodes within one process.
- **Key Concepts:** `rclcpp_components`, intra-process communication.
- **Deliverable:** Publisher & Subscriber as composable nodes loaded in a `component_container`.

---

### **Task 10: Mini Navigation Demo (Custom Simplified Nav2 Flow)**
- **Goal:** Implement a simplified navigation pipeline.
- **Key Concepts:** Multi-node system (Goal node → Planner → Controller → TF → Velocity command).
- **Deliverable:** 
  - Goal node sends target position
  - Simple planner calculates path
  - Controller publishes velocity commands
  - TF updates robot movement

---

## **How to Build**
### **Prerequisites**
- Ubuntu 22.04 or similar
- ROS2 Humble or newer
- Colcon build system

### **Build Steps**
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Clone repository
git clone https://github.com/<your-username>/ros2-hands-on-tasks.git
cd ros2-hands-on-tasks

# Build all packages
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Run an example (Task 1)
ros2 run task01_pubsub publisher
ros2 run task01_pubsub subscriber
