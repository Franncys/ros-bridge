import json
import logging
from datetime import datetime
from abc import ABC, abstractmethod
from carla_ros_bridge.FaultInjector.gnss_data import GNSSData

class FaultInjector(ABC):
    def __init__(self, sensor_name, config_file=None):
        """
        Initialize the FaultInjector with faults specific to the given sensor.

        :param config_file: Path to the fault configuration file.
        :type config_file: str
        :param sensor_name: Name of the sensor (e.g., "IMUSensor").
        :type sensor_name: str
        """
        # Set up logging
        log_file_name = f"/tmp/sensor_faultInjection_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

        self.logger = logging.getLogger(f"FaultInjector-{sensor_name}")
        self.logger.setLevel(logging.DEBUG)

        file_handler = logging.FileHandler(log_file_name)
        file_handler.setLevel(logging.DEBUG)
        
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)

        self.logger.addHandler(file_handler)

        # Initialize faults
        self.sensor_name = sensor_name
        self.faults = []
        self.active_faults = []

        # Load faults if a configuration file is provided
        if config_file:
            self._load_faults(config_file)
        else:
            self.logger.info(f"No fault configuration file provided for {sensor_name}. Fault injection disabled.")

        # with open(config_file, 'r') as f:
        #     all_faults = json.load(f)
        
        # self.faults = []
        # for fault_entry in all_faults:
        #     if fault_entry['sensor'] == sensor_name:
        #         self.faults.extend(fault_entry['faults'])
        
        # self.active_faults = []  # List of active faults with activation timestamps
        # self.logger.info(f"Initialized FaultInjector for {sensor_name} with {len(self.faults)} faults.")

    def check_and_trigger_faults(self, timestamp, carla_location):
        """
        Check if any faults should be triggered based on the current GNSS location.

        :param sensor: The sensor instance.
        :param timestamp: The current timestamp.
        """
        #log carla_location

        # current_location = GNSSData.get_location()
        # if not current_location:
        #     self.logger.warning("No GNSS location available. Skipping fault trigger check.")
        #     return
        # Log the current GNSS location
        #self.logger.info(f"Current GNSS location: {current_location} at timestamp {timestamp}.")

        # Check and trigger new faults
        for fault in self.faults:
            self.logger.info(f"Carla location: {carla_location} at timestamp {timestamp}.")
            #self.logger.info(f"Checking Fault: {json.dumps(fault, indent=2)}")  # Properly format fault details
            #self.logger.info(f"Fault Trigger: {json.dumps(fault.get('trigger', {}), indent=2)}")  # Properly format trigger details

            if self._is_triggered(fault, timestamp, carla_location):
                self.active_faults.append({
                    "fault": fault,
                    "activation_time": timestamp
                })
                self.logger.info(f"Triggered fault: {fault['name']} at timestamp {timestamp}.")

        # Remove expired faults
        self._remove_expired_faults(timestamp)

    @abstractmethod
    def apply_faults(self, sensor_data):
        """
        Abstract method to apply active faults to the sensor data.
        Must be implemented by subclasses.
        """
        pass

    def _remove_expired_faults(self, timestamp):
        """
        Remove faults from active_faults if their duration has elapsed.

        :param timestamp: The current timestamp.
        """
        before_count = len(self.active_faults)
        self.active_faults = [
            active_fault for active_fault in self.active_faults
            if timestamp - active_fault["activation_time"] < active_fault["fault"].get("duration", float("inf"))
        ]
        expired_count = before_count - len(self.active_faults)
        if expired_count > 0:
            self.logger.info(f"Removed {expired_count} expired faults at timestamp {timestamp}.")

    def _is_triggered(self, fault, timestamp, location):
        """
        Check if the fault should be triggered based on time or location.

        :param fault: The fault configuration.
        :param timestamp: The current timestamp.
        :param location: The current GNSS location.
        :return: True if the fault should be triggered, False otherwise.
        """
        if 'time' in fault['trigger'] and fault['trigger']['time'] <= timestamp:
            return True
        if 'location' in fault['trigger'] and self._is_within_location(fault['trigger']['location'], location):
            return True
        return False

    def _is_within_location(self, fault_location, current_location, tolerance=1.0):
        """
        Check if the current Carla location is approximately within the fault trigger location.

        :param fault_location: The location specified in the fault trigger (x, y, z).
        :param current_location: The current Carla location (x, y, z).
        :param tolerance: The maximum distance allowed for a match (default: 0.5 meters).
        :return: True if the current location is within the tolerance of the fault location, False otherwise.
        """
        # Log the fault location and current location
        self.logger.info(f"Fault location: {fault_location}, Current location: {current_location}")

        try:
            # Calculate the Euclidean distance between the fault location and the current location
            distance = ((fault_location['x'] - current_location.x) ** 2 +
                        (fault_location['y'] - current_location.y) ** 2 +
                        (fault_location['z'] - current_location.z) ** 2) ** 0.5

            # Log the calculated distance
            self.logger.info(f"Distance between fault location and current location: {distance:.3f} meters")

            # Check if the distance is within the tolerance
            result = distance <= tolerance
            if result:
                self.logger.info(f"Location match successful: {fault_location} within {tolerance} meters of {current_location}")
            self.logger.info(f"Location match result: {result} (tolerance: {tolerance} meters)")
            return result
        except KeyError as e:
            self.logger.error(f"KeyError in fault location or current location: {e}")
            return False
    
    def _load_faults(self, config_file):
        """
        Load faults from the specified configuration file.

        :param config_file: Path to the fault configuration file.
        :type config_file: str
        """
        
        with open(config_file, 'r') as f:
            all_faults = json.load(f)

        self.faults = []
        for fault_entry in all_faults:
            if fault_entry['sensor'] == self.sensor_name:
                self.faults.extend(fault_entry['faults'])

        self.active_faults = []  # Reset active faults
        self.logger.info(f"Loaded {len(self.faults)} faults from {config_file}.")

    def reload_faults(self, new_file):
        """
        Reload faults from a new configuration file.

        :param new_file: Path to the new fault configuration file.
        :type new_file: str
        """
        
        # TO CHANGE FOR THE MOMENT THE FILE IS HARDCODED
        new_file = '/tum/src/carla/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/FaultInjector/FaultConfigFiles/' + new_file
        
        self.logger.info(f"Reloading faults from file: {new_file}")
        print("Reloading faults from file: ", new_file)

        self._load_faults(new_file)