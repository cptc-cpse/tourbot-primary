from models.message import Message
from communication.mqtt_client import Client

LIDAR_TOPIC = "tourbot/v1/sensors/lidar/scan"
OAK_TOPIC = "tourbot/v1/sensors/oak/scan"
MAP_TOPIC = "tourbot/v1/maps/snapshot"
METERS_PER_INDEX = .1 
MAP_WIDTH_M = 10
MAP_HEIGHT_M = 10

#Map key, -1 is unknown, 0 is free, 1 is tourbot, 2 is stationary obstacle, 3 is moving obstacle


def receive_lidar(message: Message):
    # Process the received LiDAR data
    pass

def receive_OAK(message: Message):
    # Process the received OAK data
    pass

def maintain_map():
    # Maintain the map based on received data
    client = Client(qos=0, retain=False, process_id="mapping_service")
    client.subscribe(LIDAR_TOPIC, receive_lidar)
    