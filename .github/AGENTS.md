# Agents

## Context

- This is a **ROS 2** (Robot Operating System 2) C++ project for a hexapod robot.

## Code Guidelines

- Use **references** (`&` / `const&`) over raw pointers. When pointers are necessary, use smart pointers (`std::shared_ptr`, `std::unique_ptr`).
- Prefer **C++20** features where useful (e.g., `std::format`, concepts, ranges, designated initializers, `std::span`).

## Build Instructions

- Build the entire workspace:
  ```bash
  colcon build --symlink-install
  ```
- Build a single package:
  ```bash
  colcon build --packages-select <package_name>
  ```
- Run tests for a package:
  ```bash
  colcon test --packages-select <package_name> --event-handlers console_direct+
  ```
