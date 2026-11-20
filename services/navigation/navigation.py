from asyncio.log import logger
from threading import Event, Lock
import time
from communication.message import Message
from communication.mqtt_client import Client
from configuration import get_config_value
from services.hmi.hmi_interface import HMIInterface
from services.plc.plc_motor_client import PLCMotorClient

'''

This module provides the system with methods that make 
decisions on how the Tourbot should navigate in its space,
and it will then send specified movement commands to the plc interface.

'''

class NavigationService:
    def __init__(
        self, 
        hmi_interface: HMIInterface, 
        plc_motor_client: PLCMotorClient,
        mqtt_client: Client,
    ):

        # Needed to access hmi.latest_command
        self.hmi = hmi_interface
        # Needed to send movement commands to the Tourbot
        self.plc = plc_motor_client
        # Idle, Manual, or Autonomous
        self.navigation_state = "idle"
        # Once true, the tourbot stops permanently.
        self.obstacle_detected = False
        # The last command sent to the PLC interface
        self.last_sent_command = None

        # Lidar obstacle detection variables
        self._scan_lock = Lock()
        self._obstacle_ahead = False
        self._last_scan_time = 0.0
        self._scan_received = Event()
        mqtt_client.subscribe(
            get_config_value("lidar_topic"), 
            self._on_lidar_scan
        )

    def start(self):
        # Start navigation service
        logger.info("Navigation service started. Current state: %s", self.navigation_state)

        while not self._scan_received.wait(timeout=0.1):
            pass  # Don't process drive commands before LiDAR is ready.
        
        # Our tracer bullet calls for the Tourbot to move forward until an object
        # is detected, hence this flag.
        while self.obstacle_detected == False:
            if self.detect_obstacle():
                break
            self.process_hmi_input()
            time.sleep(0.05)

    def navigate(self):
            if self.navigation_state == "autonomous":
                logger.info("Entering autonomous navigation mode.")
                self.autonomous_navigation()
            elif self.navigation_state == "manual":
                logger.info("Entering manual navigation mode.")
                self.manual_navigation()
    
    def process_hmi_input(self):
        if self.hmi.latest_command == self.last_sent_command:
            pass
        elif self.hmi.latest_command in {"TAKE_CONTROL", "RELEASE_CONTROL"}:
            self.update_navigation_state()
        else:
            self.navigate()

    # Check the latest hmi command and update the navigation state accordingly
    def update_navigation_state(self):  

        if self.hmi.latest_command == "TAKE_CONTROL" and self.navigation_state != "manual":   
            # Send command and check that it was received
            if not self.plc.send_command(self.hmi.latest_command):
                logger.error("Failed to send %s command to PLC", self.hmi.latest_command)
            else:
                self.navigation_state = "manual"
                logger.info("Navigation state updated to: %s", self.navigation_state)

        elif self.hmi.latest_command == "RELEASE_CONTROL" and self.navigation_state != "idle":        
            # Send command and check that it was received
            while not self.plc.stop():
                logger.error("Failed to send STOP command to PLC")
                time.sleep(0.5)
           
            self.navigation_state = "idle"
            logger.info("Navigation state updated to: %s", self.navigation_state)

    def _on_lidar_scan(self, message: Message) -> None:
        points = message.payload
        if not isinstance(points, list):
            return

        obstacle = any(
            isinstance(p, dict)
            and isinstance(p.get("angle_deg"), (int, float))
            and isinstance(p.get("distance_mm"), (int, float))
            and abs((p["angle_deg"] + 180) % 360 - 180) <= 30
            and 0 < p["distance_mm"] <= 500
            for p in points
        )

        with self._scan_lock:
            self._obstacle_ahead = obstacle
            self._last_scan_time = message.timestamp
            self._scan_received.set()

    # Returns True if an obstacle is detected in the Tourbot's path, False otherwise
    def detect_obstacle(self) -> bool:
        with self._scan_lock:
            detected = (
                self._obstacle_ahead
                or time.time() - self._last_scan_time > 1.0
            )

        if detected and not self.obstacle_detected:
            # Send command and check that it was received
            while not self.plc.stop():
                logger.error("Failed to send STOP command to PLC")
                time.sleep(0.5)
            
            self.navigation_state = "idle"
            self.obstacle_detected = detected

        return detected

    def manual_navigation(self):

        if (self.obstacle_detected == False and 
            self.hmi.latest_command != self.last_sent_command
        ):
            # Send command and check that it was received
            if self.plc.send_command(self.hmi.latest_command) == True:
                self.last_sent_command = self.hmi.latest_command
            else:
                logger.error("Failed to send PLC command %r", self.hmi.latest_command)    



    def autonomous_navigation(self):
        # Wait for checkpoint selection from the HMI interface.

        # If a checkpoint is selected, route a path to the checkpoint.

        # If a moving object is in the Tourbot's path, stop and wait for 
        # the object to clear the path.

        # If a stationary object is in the Tourbot's path, reroute to 
        # avoid the object.

        # When the Tourbot reaches the checkpoint perform reached checkpoint 
        # routine.
         
        # While their are still unvisited checkpoints, wait for next checkpoint 
        # selection from the HMI interface.

        # If all checkpoints have been visited, return to the starting point and
        # perform completed route routine.

        pass