import os
from typing import Any

DEFAULT_CONFIG = {
    "mqtt_broker": "localhost",
    "mqtt_port": 1883,
    "mqtt_username": "",
    "mqtt_password": "",
    "lidar_port": os.environ.get(
        "LIDAR_PORT",
        "COM3" if os.name == "nt" else "/dev/ttyUSB0",
    ),
    "oak_camera_fps": 30,
    "oak_image_width": 640,
    "oak_image_height": 400,
    "oak_imu_frequency": 200,
    "lidar_publish_interval_s": 1.0,
    "oak_publish_interval_s": 1.0,
    "map_publish_interval_s": 1.0,
    "map_resolution_m": 0.1,
    "lidar_topic": "tourbot/v1/sensors/lidar/scan",
    "oak_topic": "tourbot/v1/sensors/oak/scan",
    "map_topic": "tourbot/v1/maps/snapshot",
    "hmi_command_topic": "tourbot/dashboard/manualControl",
    "hmi_response_topic": "tourbot/dashboard/pcResponse"
}


def get_config_value(key: str) -> Any:
    """Retrieve a configuration value with a fallback to the default."""
    try:
        return DEFAULT_CONFIG.get(key)
    except KeyError:
        raise ValueError(f"Configuration key not found: {key}")