'''

This module contains the HMIInterface class, which provides 
an interface for interacting with the Node-RED Dshboard on the 
(HMI) of the system. The HMIInterface class receives commands 
and publishes a response 

Input:
- Manual input from the user through the Node-RED Dashboard

Output:
- Response to the user through the Node-RED Dashboard

'''

import logging
import sys
from pathlib import Path
from threading import Event

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from configuration import get_config_value
from communication.mqtt_client import Client
from communication.message import Message

logger = logging.getLogger(__name__)


class HMIInterface:
    def __init__(self, node_red_client: Client):
        self.node_red_client = node_red_client
        # Drive or state commands that are accesed by the navigation service
        self.latest_command = None

    def start(self) -> None:
        """Subscribe to the command topic used by the dashboard."""
        self.node_red_client.subscribe(
            get_config_value("hmi_command_topic"),
            self.receive_command,
        )

    def receive_command(self, message: Message) -> None:
        """Log a dashboard command and publish an acknowledgement."""
        logger.info(
            "Received MQTT command on %s: %r",
            get_config_value("hmi_command_topic"),
            message.payload,
        )
        response = self.process_command(message.payload)
        self.publish_response(response)
        self.latest_command = response


    @staticmethod
    def process_command(command: object) -> str:
        """Return a simple acknowledgement; command execution is not handled here."""
        if not isinstance(command, str) or not command.strip():
            return " Empty or invalid command"
        return command.strip()

    def publish_response(self, response: str) -> None:
        """Publish a plain-text response for the dashboard's MQTT input node."""
        self.node_red_client.publish(get_config_value("hmi_response_topic"), response)
        logger.info(
            "Published MQTT response on %s: %s",
            get_config_value("hmi_response_topic"),
            response,
        )


'''

This code snippet is a standalone script that initializes the HMIInterface and starts 
listening for commands from the Node-RED Dashboard directly from this file. It is 
intended for testing and debugging purposes, allowing developers to run the 
HMI interface without needing to start the entire system.

'''
def main() -> None:
    logging.basicConfig(level=logging.INFO)
    with Client(qos=1, retain=False, process_id="hmi") as client:
        HMIInterface(client).start()
        logger.info(
            "Listening for dashboard commands on %s",
            get_config_value("hmi_command_topic"),
        )
        Event().wait()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Stopping HMI interface")