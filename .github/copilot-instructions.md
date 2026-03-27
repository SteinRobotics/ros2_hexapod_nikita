# GitHub Copilot Instructions

## Build Instructions

- This is a ROS2 project, use colcon commands for building and testing like:
- `colcon build --symlink-install` 
- `colcon build --packages-select <package_name>` 
- `colcon test --packages-select <package_name> && colcon test-result --verbose`
- `colcon test --packages-select <package_name> --event-handlers console_direct+`

## Code Instructions

- Use **references** (`&` / `const&`) over raw pointers. When pointers are necessary, use smart pointers (`std::shared_ptr`, `std::unique_ptr`).
- Prefer **C++20** features where useful (e.g., `std::format`, concepts, ranges, designated initializers, `std::span`).
