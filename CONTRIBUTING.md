# Contributing to NuWave ROV

Thanks for your interest in contributing to the Northeastern underwater robotics team's ROV software.
This guide gives a brief overview of the project and how its code is organized.

For development environment setup, see the [software setup guide](software_setup.md). For coding and
Git conventions, see the [software development best practices](best_practices.md).

## Project Overview

This repository contains the ROS 2 software used to operate and support the team's remotely operated
vehicle (ROV). It is organized as a `colcon` workspace of ROS 2 packages, with nodes running both on
the topside laptop and on the ROV's onboard computer. The software covers operator input and control,
vehicle actuation, sensing, stabilization, simulation, and a web interface.

## Project Structure

The project is organized as ROS 2 packages under `src/`. A typical package contains a Python code
package, a `test/` folder, and a `config/` folder when it needs configuration. The code package
contains node entry points and supporting methods; exact files vary by package.

```text
src/
└── example_package/
    ├── config/                      # Optional package configuration
    ├── test/                         # Package tests
    │   ├── example_node_test.py      # Tests for the ROS 2 node
    │   └── example_functions_test.py    # Tests for supporting methods
    ├── example_package/              # Python code package
    │   ├── example_node.py           # ROS 2 node and entry point
    │   └── example_functions.py        # Supporting methods
    ├── package.xml
    └── setup.py
```

## Architecture and Design Principles

- **Use ROS 2 packages and nodes:** Keep related functionality within its node, and have nodes
  communicate through ROS 2 interfaces.
- **Separate methods from the node:** Define methods used by a node in the methods file. Pass any
  required node information to each method as parameters.
- **Separate configuration from code:** Prefer package configuration files for tunable values and
  hardware-specific settings.
- **Write tests for all new methods:** Ensure each method has tests that cover all relevant behavior.
- **Follow existing conventions:** Use the repository's development and Git guidance, and update
  relevant documentation when behavior or setup changes.

## Testing

## Dependencies

## Pull Requests

## Issue Reporting