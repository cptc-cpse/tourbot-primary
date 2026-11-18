import time

from communication.message import Message
from communication.mqtt_client import Client
from configuration import get_config_value
from services.mapping.mapping_state import MappingState

LIDAR_TOPIC: str = get_config_value("lidar_topic")
OAK_TOPIC: str = get_config_value("oak_topic")
MAP_TOPIC: str = get_config_value("map_topic")
PUBLISH_INTERVAL: float = get_config_value("map_publish_interval_s")

def maintain_map() -> None:
    """Maintain a map by subscribing to LIDAR and OAK topics and publishing snapshots."""
    client = Client(qos=0, retain=False, process_id="mapping_service")
    state = MappingState()

    client.subscribe(LIDAR_TOPIC, state.receive_lidar)
    client.subscribe(OAK_TOPIC, state.receive_oak)

    while True:
        payload = state.get_map_snapshot()
        if payload is not None:
            client.publish(MAP_TOPIC, Message(payload).to_json())
        time.sleep(PUBLISH_INTERVAL)
