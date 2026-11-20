import time
import threading

from communication.mqtt_client import Client
from configuration import get_config_value
from communication.message import Message
import services.oak.oak_data_collection as oak
import services.lidar.lidar_data_collection as lidar
from services.hmi.hmi_interface import HMIInterface
import services.mapping.mapping as mapping
from services.navigation.navigation import NavigationService
from services.plc.plc_motor_client import PLCMotorClient

def test_subscribe(message: Message) -> None:
    print("callback received: ", message.payload)

def main():
    client = Client(qos=0, retain=False, process_id="main")
    hmi_interface = HMIInterface(client)
    hmi_interface.start()

    plc = PLCMotorClient(host="192.168.0.2", port=5005)
    plc.connect()

    navigation_service = NavigationService(hmi_interface, plc)
    navigationThread = threading.Thread(target=navigation_service.start)
    navigationThread.start()

    #client.subscribe(get_config_value("oak_topic"), test_subscribe)
    oakThread = threading.Thread(target=oak.stream_data)
    oakThread.start()

    #client.subscribe(get_config_value("lidar_topic"), test_subscribe)
    lidarThread = threading.Thread(target=lidar.stream_data)
    lidarThread.start()

    #client.subscribe(get_config_value("map_topic"), test_subscribe)
    mapThread = threading.Thread(target=mapping.maintain_map)
    mapThread.start()


if __name__ == "__main__":
    main()