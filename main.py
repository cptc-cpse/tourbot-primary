import time

from communication.mqtt_client import Client
from models.message import Message
import services.oak_data_collection as oak
import services.lidar_data_collection as lidar

def test_subscribe(message: Message) -> None:
    print("callback received: ", message.payload)

def main():
    client = Client(qos=0, retain=False, process_id="main")
    #client.subscribe(oak.POSE_TOPIC, test_subscribe)
    #client.subscribe(oak.OBSTACLE_POINT_CLOUD_TOPIC, test_subscribe)
    #client.subscribe(oak.GROUND_POINT_CLOUD_TOPIC, test_subscribe)
    #client.subscribe(oak.OCCUPANCY_GRID_TOPIC, test_subscribe)
    #oak.stream_data()

    client.subscribe(lidar.TOPIC, test_subscribe)
    lidar.stream_data()


if __name__ == "__main__":
    main()