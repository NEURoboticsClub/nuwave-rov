import os
import yaml

def load_yaml(config_path: str, error_header: str = "") -> dict:
    if not os.path.exists(config_path):
        print(f"{error_header} Config file not found: {config_path}")
        return {}
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        print(f"{error_header} Failed to parse {config_path}: {e}")
        return {}

# def load_yaml(self, path):
#     """Load a YAML file from a relative or absolute path."""
#     if not os.path.exists(path):
#         self.get_logger().warn(f"Config file not found: {path}")
#         return {}
#     with open(path, 'r') as f:
#         return yaml.safe_load(f)