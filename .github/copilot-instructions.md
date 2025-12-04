# GitHub Copilot Instructions

## Build Instructions

- This is a ROS2 project, use colcon commands for building and testing like:
- `colcon build --symlink-install` 
- `colcon build --packages-select <package_name>` 
- `colcon test --packages-select <package_name> && colcon test-result --verbose`
- `colcon test --packages-select <package_name> --event-handlers console_direct+`

## Code Instructions

- Use smart pointers over raw pointers
- Use C++20 functions where useful
