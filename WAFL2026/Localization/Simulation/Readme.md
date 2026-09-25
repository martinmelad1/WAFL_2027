# Autonomous Forklift Simulation Localization

This workspace contains the ROS 2 packages for the WAFL 2026 project's localization.

## Expected Workspace Structure

To ensure all packages compile correctly, organize your local ROS 2 workspace exactly as shown below:

```text
localization_ws/
└── src/
    ├── rplidar_ros     # 2D LiDAR driver and scanning nodes
    └── wafl2026        # Core configuration, navigation stacks, & launch files

