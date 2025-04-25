from carla_ros_bridge.FaultInjector.FaultInjector import FaultInjector
from transforms3d.euler import euler2quat
import math

class IMUFaultInjector(FaultInjector):
    def __init__(self, config_file):
        super().__init__(config_file, "IMUSensor")

    def apply_faults(self, sensor_data):
        """
        Apply IMU-specific faults to the sensor data.
        """
        for fault in self.active_faults:
            if fault['failure_type'] == 'bias':
                sensor_data = self._apply_bias(sensor_data, fault)
            elif fault['failure_type'] == 'noise':
                sensor_data = self._apply_noise(sensor_data, fault)
            elif fault['failure_type'] == 'dropout':
                sensor_data = self._apply_dropout(sensor_data, fault)
            elif fault['failure_type'] == 'rotation':
                sensor_data = self._apply_rotation(sensor_data, fault)
        return sensor_data

    def _apply_rotation(self, sensor_data, fault):
        """
        Simulate a rotation by modifying the orientation quaternion based on the fault parameters.

        :param sensor_data: The IMU sensor data to modify.
        :param fault: The fault configuration containing rotation parameters.
        """
        # Extract rotation parameters from the fault configuration
        axis = fault.get('parameters', {}).get('axis', 'z')  # Default to Z-axis
        angle_degrees = fault.get('parameters', {}).get('angle', 180)  # Default to 180º
        angle_radians = math.radians(angle_degrees)  # Convert to radians

        # Determine the rotation based on the specified axis
        if axis == 'x':
            roll, pitch, yaw = angle_radians, 0.0, 0.0
        elif axis == 'y':
            roll, pitch, yaw = 0.0, angle_radians, 0.0
        elif axis == 'z':
            roll, pitch, yaw = 0.0, 0.0, angle_radians
        else:
            raise ValueError(f"Invalid axis '{axis}' specified in fault configuration. Must be 'x', 'y', or 'z'.")

        # Convert the rotation to a quaternion
        quat = euler2quat(roll, pitch, yaw)  # Convert to quaternion

        # Apply the rotation to the sensor data's orientation
        sensor_data.orientation.w = quat[0]
        sensor_data.orientation.x = quat[1]
        sensor_data.orientation.y = quat[2]
        sensor_data.orientation.z = quat[3]

        return sensor_data