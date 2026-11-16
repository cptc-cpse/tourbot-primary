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
current_position: dict[str, float] | None = None

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
        resolution_m = payload["resolution_m"]
        position = payload["position"]
        orientation = payload["orientation"]
        print(orientation)
        
        x_index = int((position["x"] - map_origin["x"]) / resolution_m)
        y_index = int((position["y"] - map_origin["y"]) / resolution_m)
        current_position = {"x": x_index, "y": y_index, "orientation": orientation}

        for x in range(map_width):
            for y in range(map_height):
                if oak_map[y][x] >= 10:  # Assuming values >= 10 are probably obstacles
                    oak_map[y][x] = 2  # Mark as stationary obstacle
                elif x == x_index and y == y_index:
                    oak_map[y][x] = 1  # Mark as tourbot's position
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
        publish_map()
        time.sleep(PUBLISH_INTERVAL)


def publish_map():
    global occupancy_map, current_position
    if occupancy_map is not None:
        payload = {
            "map": occupancy_map,
            "resolution_m": MAP_RESOLUTION_M,
            "position": current_position
        }
        client = Client(qos=0, retain=False, process_id="mapping_service")
        client.publish(MAP_TOPIC, Message(payload).to_json())