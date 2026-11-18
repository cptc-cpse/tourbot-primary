import math
import time

from models.message import Message
from communication.mqtt_client import Client
from configuration import get_config_value

LIDAR_TOPIC: str = get_config_value("lidar_topic")
OAK_TOPIC: str = get_config_value("oak_topic")
MAP_TOPIC: str = get_config_value("map_topic")
MAP_RESOLUTION_M: float = get_config_value("map_resolution_m")
PUBLISH_INTERVAL: float = get_config_value("map_publish_interval_s")
# I took these measurements with a 6 inch ruler, so they may not be exact, and I rounded the base measurements.
# The base is the widest part of the robot, and the body is the narrower part that sits on top of the base.
TOURBOT_BASE_WIDTH_M: float = 0.5  # Actually measured 53 cm
TOURBOT_BASE_LENGTH_M: float = 0.6  # Actually measured 62 cm
TOURBOT_BODY_WIDTH_M: float = 0.5 
TOURBOT_BODY_LENGTH_M: float = 0.3 # Should be the distance between the OAK cameras

#Map key, -1 is unknown, 0 is free, 1 is tourbot, 2 is stationary obstacle, 3 is moving obstacle
occupancy_map: list[list[int]] | None = None
current_position: dict[str, int | float] | None = None

def receive_lidar(message: Message):
    # Process the received LiDAR data
    pass

def receive_OAK(message: Message):
    global occupancy_map, current_position
    # Process the received OAK data
    payload = message.payload
    try:
        oak_map = payload["map"]
        map_origin = payload["map_origin"]
        map_width = payload["map_width"]
        map_height = payload["map_height"]
        position = payload["position"]
        orientation = payload["orientation"]
        print(position)

        if (position["x"] == 0 and position["y"] == 0 and position["z"] == 0) and (orientation["x"] == 0.5 and orientation["y"] == 0.5 and orientation["z"] == 0.5 and orientation["w"] == 0.5):
            print("Default positional values detected. OAK odometry likely stalled.")
            return
        
        x_index = int((position["x"] - map_origin["x"]) / MAP_RESOLUTION_M)
        y_index = int((position["y"] - map_origin["y"]) / MAP_RESOLUTION_M)
        heading_degrees = quaternion_to_map_heading_deg(orientation)
        current_position = {"x": x_index, "y": y_index, "orientation": heading_degrees}

        print(heading_degrees)

        for x in range(map_width):
            for y in range(map_height):
                if x == x_index and y == y_index:
                    oak_map[y][x] = 1  # Mark as tourbot's position
                elif oak_map[y][x] >= 10:  # Assuming values >= 10 are probably obstacles
                    oak_map[y][x] = 2  # Mark as stationary obstacle
                else:
                    oak_map[y][x] = 0  # Mark as free space

        occupancy_map = oak_map


    except KeyError:
        print("Received OAK message without required keys.")
        return
    

def maintain_map():
    # Maintain the map based on received data
    client = Client(qos=0, retain=False, process_id="mapping_service")
    client.subscribe(LIDAR_TOPIC, receive_lidar)
    client.subscribe(OAK_TOPIC, receive_OAK)
    # Normally the map would be initialized here

    while True:
        publish_map(client)
        time.sleep(PUBLISH_INTERVAL)


def publish_map(client: Client) -> None:
    global occupancy_map, current_position
    if occupancy_map is not None:
        payload = {
            "map": occupancy_map,
            "resolution_m": MAP_RESOLUTION_M,
            "position": current_position
        }
        client.publish(MAP_TOPIC, Message(payload).to_json())

def quaternion_to_map_heading_deg(orientation: dict[str, float]) -> float:
    """Convert an FLU-frame quaternion to clockwise map heading from -Y."""
    x, y, z, w = (
        orientation["x"],
        orientation["y"],
        orientation["z"],
        orientation["w"],
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("Orientation quaternion must have a finite, non-zero norm")

    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    heading_degrees = (90 - math.degrees(yaw)) % 360.0
    if math.isclose(heading_degrees, 360.0, rel_tol=0.0, abs_tol=1e-12):
        return 0.0
    return heading_degrees