import time

from models.message import Message
from communication.mqtt_client import Client
import communication.mqtt_topics as topics

LIDAR_TOPIC = topics.LIDAR_TOPIC
OAK_TOPIC = topics.OAK_TOPIC
MAP_TOPIC = topics.MAP_TOPIC
METERS_PER_INDEX = .1 
PUBLISH_INTERVAL = 1.0  # seconds

#Map key, -1 is unknown, 0 is free, 1 is tourbot, 2 is stationary obstacle, 3 is moving obstacle
occupancy_map = None
current_position = None

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
        
        current_position = position
        x_index = int((position["x"] - map_origin["x"]) / resolution_m)
        y_index = int((position["y"] - map_origin["y"]) / resolution_m)

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
        time.sleep(PUBLISH_INTERVAL)  # Adjust the sleep duration as needed

def publish_map():
    global occupancy_map, current_position
    if occupancy_map is not None:
        payload = {
            "map": occupancy_map,
            "resolution_m": METERS_PER_INDEX,
            "position": current_position
        }
        client = Client(qos=0, retain=False, process_id="mapping_service")
        client.publish(MAP_TOPIC, Message(payload).to_json())