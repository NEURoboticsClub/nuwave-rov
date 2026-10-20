# Shared NUWave utilities

This package provides Python helpers used by Houston, the arm and thruster
controllers, the PWM node, and stabilization. It is a library package with no
node executable, publishers, or subscribers.

## YAML configuration helper

```python
from nuwave_utils_pkg.file_helpers import load_yaml

config = load_yaml('/path/to/config.yaml', error_header='[controller]')
motors = config.get('motors', [])
```

`load_yaml(config_path, error_header="")` parses a file with `yaml.safe_load`.
Missing files, malformed YAML, and empty documents return an empty dictionary.
Callers must validate required fields and the root value's type before using
configuration data: non-empty YAML sequences or scalars can also be returned.
`error_header` supplies caller context for reported configuration errors.

## Setup and dependencies

Build and source the workspace before importing the installed package:

```sh
colcon build --packages-select nuwave_utils_pkg
source install/setup.bash
```

The helper uses PyYAML and Python's standard library; ROS 2 `rclpy` is declared
as a package dependency. Consumers should declare this package as a dependency.
The package currently has no ROS parameters or hardware requirements.
See the [helper source](nuwave_utils_pkg/file_helpers.py),
[software setup](../../software_setup.md), and
[PyYAML documentation](https://pyyaml.org/wiki/PyYAMLDocumentation).
