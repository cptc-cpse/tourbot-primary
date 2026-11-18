import time
import threading

from communication.mqtt_client import Client
from configuration import get_config_value
from models.message import Message
import services.oak_data_collection as oak
import services.lidar_data_collection as lidar
import services.mapping as mapping

def test_subscribe(message: Message) -> None:
    print("callback received: ", message.payload)

def main():
    client = Client(qos=0, retain=False, process_id="main")
    hmi_interface = HMIInterface(client)
    hmi_interface.start()

    #client.subscribe(get_config_value("oak_topic"), test_subscribe)
    oakThread = threading.Thread(target=oak.stream_data)
    oakThread.start()

    #client.subscribe(get_config_value("lidar_topic"), test_subscribe)
    #lidarThread = threading.Thread(target=lidar.stream_data)
    #lidarThread.start()

    #client.subscribe(get_config_value("map_topic"), test_subscribe)
    mapThread = threading.Thread(target=mapping.maintain_map)
    mapThread.start()


if __name__ == "__main__":
    main()