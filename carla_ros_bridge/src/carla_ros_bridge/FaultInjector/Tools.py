import json

def has_fault_for_sensor(config_file, sensor_name):
    """
    Check if the configuration file contains faults for the given sensor.

    :param config_file: Path to the fault configuration file.
    :type config_file: str
    :param sensor_name: Name of the sensor to check for faults.
    :type sensor_name: str
    :return: True if faults exist for the sensor, False otherwise.
    :rtype: bool
    """
    try:
        with open(config_file, 'r') as f:
            faults = json.load(f)
            for fault_entry in faults:
                if fault_entry.get("sensor") == sensor_name:
                    return True
    except Exception as e:
        print(f"Error reading fault configuration file: {e}")
    return False