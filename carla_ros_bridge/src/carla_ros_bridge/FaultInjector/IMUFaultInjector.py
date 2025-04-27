from carla_ros_bridge.FaultInjector.FaultInjector import FaultInjector
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat
import math

class IMUFaultInjector(FaultInjector):
    def __init__(self, config_file):
        super().__init__(config_file, "IMUSensor")

    def apply_faults(self, sensor_data):
        """
        Apply IMU-specific faults to the sensor data.
        """
        try:
            for active_fault in self.active_faults:
                fault = active_fault["fault"] 
                # Log sensor data before applying faults
                self.logger.info("IMU Sensor data before applying faults: %s", sensor_data)

                self.logger.info(f"Applying fault: {fault['name']} with parameters: {fault['parameters']}")
                if fault['name'] == 'bias':
                    sensor_data = self._apply_bias(sensor_data, fault)
                elif fault['name'] == 'noise':
                    sensor_data = self._apply_noise(sensor_data, fault)
                elif fault['name'] == 'dropout':
                    sensor_data = self._apply_dropout(sensor_data, fault)
                elif fault['name'] == 'rotation':
                    sensor_data = self._apply_rotation(sensor_data, fault)
                # Log sensor data after applying faults
                self.logger.info("IMU Sensor data after applying faults: %s", sensor_data)
                return sensor_data    
            
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying faults to IMU data: {e}")
            return sensor_data

    def _apply_rotation(self, sensor_data, fault):
        """
        Simulate a rotation by modifying the orientation quaternion based on the fault parameters.

        :param sensor_data: The IMU sensor data to modify.
        :param fault: The fault configuration containing rotation parameters.
        """
        try:
            # Extract rotation parameters from the fault configuration
            axis = fault.get('parameters', {}).get('axis', 'z')  # Default to Z-axis
            angle_degrees = fault.get('parameters', {}).get('angle', 180)  # Default to 180º
            angle_radians = math.radians(angle_degrees)  # Convert to radians

            # Determine the rotation matrix for the specified axis
            if axis == 'x':
                rotation_matrix = euler2mat(angle_radians, 0.0, 0.0, axes='sxyz')
            elif axis == 'y':
                rotation_matrix = euler2mat(0.0, angle_radians, 0.0, axes='sxyz')
            elif axis == 'z':
                rotation_matrix = euler2mat(0.0, 0.0, angle_radians, axes='sxyz')
            else:
                raise ValueError(f"Invalid axis '{axis}' specified in fault configuration. Must be 'x', 'y', or 'z'.")

            # Log the rotation being applied
            self.logger.info(f"Applying rotation: axis={axis}, angle={angle_degrees} degrees (radians: {angle_radians})")

            # Log the original orientation
            self.logger.info(f"Original orientation: w={sensor_data.orientation.w}, x={sensor_data.orientation.x}, y={sensor_data.orientation.y}, z={sensor_data.orientation.z}")

            # Convert the current orientation quaternion to a rotation matrix
            current_quat = [
                sensor_data.orientation.w,
                sensor_data.orientation.x,
                sensor_data.orientation.y,
                sensor_data.orientation.z,
            ]
            current_rotation_matrix = quat2mat(current_quat)

            # Combine the current rotation with the new rotation
            combined_rotation_matrix = current_rotation_matrix @ rotation_matrix

            # Convert the combined rotation matrix back to a quaternion
            combined_quat = mat2quat(combined_rotation_matrix)

            # Apply the combined quaternion to the sensor data's orientation
            sensor_data.orientation.w = combined_quat[0]
            sensor_data.orientation.x = combined_quat[1]
            sensor_data.orientation.y = combined_quat[2]
            sensor_data.orientation.z = combined_quat[3]

            # Log the new orientation
            self.logger.info(f"New orientation after rotation: w={sensor_data.orientation.w}, x={sensor_data.orientation.x}, y={sensor_data.orientation.y}, z={sensor_data.orientation.z}")

            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying rotation: {e}")
            return sensor_data
