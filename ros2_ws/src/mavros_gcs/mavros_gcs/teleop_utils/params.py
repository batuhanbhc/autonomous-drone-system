import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_teleop_yaml_from_pkg(
    pkg: str = "mavros_gcs_config",
    rel: str = "config/teleop_params.yaml",
) -> dict:
    yaml_path = os.path.join(get_package_share_directory(pkg), rel)
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get("teleop", {})