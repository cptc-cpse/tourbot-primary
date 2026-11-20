from asyncio.log import logger
from services.hmi.hmi_interface import HMIInterface
from services.plc.plc_motor_client import PLCMotorClient


'''

This module provides the system with methods that make 
decisions on how the Tourbot should navigate in its space,
and it will then send specified movement commands to the plc interface.

'''

class NavigationService:
    def __init__(self, hmi_interface: HMIInterface, plc_motor_client: PLCMotorClient):
        # Needed to access hmi.latest_command
        self.hmi = hmi_interface
        # Idle, Manual, or Autonomous
        self.navigation_state = "idle"
        # Once true, the tourbot stops permanently.
        self.obstacle_detected = False
        # The last command sent to the PLC interface
        self.last_command = None

    def start(self):
        # Start navigation service
        logger.info("Navigation service started. Current state: %s", self.navigation_state)
        
        # Our tracer bullet calls for the Tourbot to move forward until an object
        # is detected, hence this flag.
        while self.obstacle_detected == False:
            
            self.update_navigation_state()

            if self.navigation_state == "autonomous":
                logger.info("Entering autonomous navigation mode.")
                self.autonomous_navigation()
            elif self.navigation_state == "manual":
                logger.info("Entering manual navigation mode.")
                self.manual_navigation()
            else:
                # Idle state, waiting for a command from the HMI interface
                pass

    # Check the latest hmi command and update the navigation state accordingly
    def update_navigation_state(self):     
        if self.hmi.latest_command == "TAKE_CONTROL":
            self.navigation_state = "manual"
            logger.info("Navigation state updated to: %s", self.navigation_state)
        elif self.hmi.latest_command == "RELEASE_CONTROL":
            self.navigation_state = "idle"
            logger.info("Navigation state updated to: %s", self.navigation_state)

    # Returns True if an obstacle is detected in the Tourbot's path, False otherwise
    def detect_obstacle(self) -> bool:
        # If obstacle is detected, send stop command to PLC interface,
        # set navigation state to idle, update self.obstacle_detected.
       
        pass

    def manual_navigation(self):

        # If detect_obstacle() returns false and 
        # self.hmi.latest_command != self.last_command:
        #     send the command to the PLC interface.
        #     update self.last_command to the latest command.


        pass

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