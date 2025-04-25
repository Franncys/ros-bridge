import json
from abc import ABC, abstractmethod
from carla_ros_bridge.FaultInjector.gnss_data import GNSSData

class FaultInjector(ABC):
    def __init__(self, config_file, sensor_name):
        """
        Initialize the FaultInjector with faults specific to the given sensor.

        :param config_file: Path to the fault configuration file.
        :type config_file: str
        :param sensor_name: Name of the sensor (e.g., "IMUSensor").
        :type sensor_name: str
        """
        with open(config_file, 'r') as f:
            all_faults = json.load(f)
        
        # Filter faults for the specific sensor
        self.faults = []
        for fault_entry in all_faults:
            if fault_entry['sensor'] == sensor_name:
                self.faults.extend(fault_entry['faults'])
        
        self.active_faults = []  # List of active faults with activation timestamps

    def check_and_trigger_faults(self, sensor, timestamp):
        """
        Check if any faults should be triggered based on the current GNSS location.

        :param sensor: The sensor instance.
        :param timestamp: The current timestamp.
        """
        current_location = GNSSData.get_location()
        if not current_location:
            return  # No GNSS data available yet

        # Check and trigger new faults
        for fault in self.faults:
            if self._is_triggered(fault, timestamp, current_location):
                # Add fault to active_faults with activation timestamp
                self.active_faults.append({
                    "fault": fault,
                    "activation_time": timestamp
                })

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
        self.active_faults = [
            active_fault for active_fault in self.active_faults
            if timestamp - active_fault["activation_time"] < active_fault["fault"].get("duration", float("inf"))
        ]

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

    def _is_within_location(self, fault_location, current_location):
        """
        Check if the current location matches the fault trigger location.

        :param fault_location: The location specified in the fault trigger.
        :param current_location: The current GNSS location.
        :return: True if the current location matches the fault location, False otherwise.
        """
        return fault_location == current_location