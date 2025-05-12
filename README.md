# Robotics Projects Repository

Welcome to the **Robotics Projects** repository! This is a centralized collection of various robotic system implementations, ranging from autonomous navigation to manual control. Each project is organized as a submodule under the `codes/` directory.

## Projects Overview

- **AOVV (Autonomous Obstacle Avoiding Vacuum Cleaner)**
  - An autonomous vacuum robot using ROS2 Nav2 for SLAM and path planning
  - Integrated sensors: BNO085 IMU, TCRT5000 line sensors, LiDAR A2M8, various limit switches
  - Motor control via TB6612FNG and expanders (MCP23017)

- **Manual-Controlled Drone**
  - Custom quadcopter platform
  - Manual remote control via RF link
  - ESCs and brushless motors integration

- **Line Follower Robot**
  - Follows ground lines using infrared matrix sensor QTR-8RC
  - PID control for smooth path tracking
  - Modular chassis and 3D-printed parts

- **Robotic Arm**
  - 4-DOF manipulator
  - Servo motor control with inverse kinematics
  - Grabber end-effector for object manipulation

## Getting Started

### Prerequisites

- Git >= 2.30
- ROS2 (Foxy or later)
- Python 3.8+
- C++17 compiler (GCC or Clang)

### Submodule Initialization

Clone the main repository and initialize all submodules:
```bash
git clone git@github.com:Krzysiek-Mistrz/electronics_robotics_designs.git
cd electronics_robotics_designs
git submodule update --init --recursive
```

## Usage

You just need to open certain folders and play those python files / cpp files ;)  

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/YourFeature`
3. Commit your changes: \`git commit -m "Add your feature"
4. Push to the branch: `git push origin feature/YourFeature`
5. Open a Pull Request and describe your changes.

## License

GNU GPL V3 @ Krzychu 2025