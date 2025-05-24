import numpy as np
from carla_ros_bridge.FaultInjector.FaultInjector import FaultInjector
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat
import math

class IMUFaultInjector(FaultInjector):
    def __init__(self, config_file=None):
        super().__init__("IMUSensor", config_file)

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
                elif fault['name'] == 'zero_value':
                    sensor_data = self._apply_zero_value(sensor_data, fault)
                elif fault['name'] == 'velocity_reduction':
                    sensor_data = self._apply_velocity_reduction(sensor_data, fault)
                elif fault['name'] == 'orientation_noise':
                    sensor_data = self._apply_orientation_noise(sensor_data, fault)
                elif fault['name'] == 'orientation_bias':
                    sensor_data = self._apply_orientation_bias(sensor_data, fault)
                elif fault['name'] == 'gyroscope_noise':
                    sensor_data = self._apply_gyroscope_noise(sensor_data, fault)
                elif fault['name'] == 'accelerometer_noise':
                    sensor_data = self._apply_accelerometer_noise(sensor_data, fault)
                elif fault['name'] == 'gyroscope_bias':
                    sensor_data = self._apply_gyroscope_bias(sensor_data, fault)
                elif fault['name'] == 'accelerometer_bias':
                    sensor_data = self._apply_accelerometer_bias(sensor_data, fault)

                # Log sensor data after applying faults
                self.logger.info("IMU Sensor data after applying faults: %s", sensor_data)
                return sensor_data    
            
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying faults to IMU data: {e}")
            return sensor_data
        
    def _apply_noise(self, sensor_data, fault):
        """
        Add Gaussian noise to IMU angular velocity and linear acceleration.
        """
        try:
            noise_stddev = fault.get('parameters', {}).get('noise_stddev', {
                "x": 0.01,
                "y": 0.01,
                "z": 0.01
            })

            # Add noise to angular velocity
            sensor_data.angular_velocity.x += np.random.normal(0, noise_stddev.get("x", 0.01))
            sensor_data.angular_velocity.y += np.random.normal(0, noise_stddev.get("y", 0.01))
            sensor_data.angular_velocity.z += np.random.normal(0, noise_stddev.get("z", 0.01))

            # Add noise to linear acceleration
            sensor_data.linear_acceleration.x += np.random.normal(0, noise_stddev.get("x", 0.01))
            sensor_data.linear_acceleration.y += np.random.normal(0, noise_stddev.get("y", 0.01))
            sensor_data.linear_acceleration.z += np.random.normal(0, noise_stddev.get("z", 0.01))

            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying noise to IMU data: {e}")
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
        
    def _apply_zero_value(self, sensor_data, fault):
        """
        Simulate a "0 value" fault by setting IMU data to zero.
        """
        try:
            sensor_data.angular_velocity.x = 0.0
            sensor_data.angular_velocity.y = 0.0
            sensor_data.angular_velocity.z = 0.0
            sensor_data.linear_acceleration.x = 0.0
            sensor_data.linear_acceleration.y = 0.0
            sensor_data.linear_acceleration.z = 0.0
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying zero value fault: {e}")
            return sensor_data
    
    def _apply_velocity_reduction(self, sensor_data, fault):
        """
        Simulate reduced velocity by scaling down angular velocity and linear acceleration.

        :param sensor_data: The IMU sensor data to modify.
        :param fault: The fault configuration containing reduction parameters.
        """
        try:
            # Get the reduction factor from the fault parameters (default to 0.5)
            reduction_factor = fault.get('parameters', {}).get('reduction_factor', 0.5)
            if not (0.0 < reduction_factor < 1.0):
                raise ValueError("Reduction factor must be between 0 and 1.")

            # Log the reduction being applied
            self.logger.info(f"Applying velocity reduction with factor: {reduction_factor}")

            # Scale down angular velocity
            sensor_data.angular_velocity.x *= reduction_factor
            sensor_data.angular_velocity.y *= reduction_factor
            sensor_data.angular_velocity.z *= reduction_factor

            # Scale down linear acceleration
            sensor_data.linear_acceleration.x *= reduction_factor
            sensor_data.linear_acceleration.y *= reduction_factor
            sensor_data.linear_acceleration.z *= reduction_factor

            # Log the modified sensor data
            self.logger.info(f"Reduced angular velocity: {sensor_data.angular_velocity}")
            self.logger.info(f"Reduced linear acceleration: {sensor_data.linear_acceleration}")

            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying velocity reduction: {e}")
            return sensor_data
        
    def _apply_orientation_noise(self, sensor_data, fault):
        """
        Add random noise to the IMU orientation quaternion.
        """
        try:
            noise_stddev = fault.get('parameters', {}).get('noise_stddev', 0.01)  # radians
            # Generate small random rotation angles
            noise_angles = np.random.normal(0, noise_stddev, 3)
            noise_rot = euler2mat(*noise_angles, axes='sxyz')
            # Convert current quaternion to rotation matrix
            current_quat = [
                sensor_data.orientation.w,
                sensor_data.orientation.x,
                sensor_data.orientation.y,
                sensor_data.orientation.z,
            ]
            current_rot = quat2mat(current_quat)
            # Apply noise
            new_rot = current_rot @ noise_rot
            new_quat = mat2quat(new_rot)
            # Update orientation
            sensor_data.orientation.w = new_quat[0]
            sensor_data.orientation.x = new_quat[1]
            sensor_data.orientation.y = new_quat[2]
            sensor_data.orientation.z = new_quat[3]
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying orientation noise: {e}")
            return sensor_data

    def _apply_orientation_bias(self, sensor_data, fault):
        """
        Add a fixed bias (rotation) to the IMU orientation quaternion.
        """
        try:
            axis = fault.get('parameters', {}).get('axis', 'z')
            angle_degrees = fault.get('parameters', {}).get('angle', 5.0)
            angle_radians = math.radians(angle_degrees)
            if axis == 'x':
                bias_rot = euler2mat(angle_radians, 0.0, 0.0, axes='sxyz')
            elif axis == 'y':
                bias_rot = euler2mat(0.0, angle_radians, 0.0, axes='sxyz')
            elif axis == 'z':
                bias_rot = euler2mat(0.0, 0.0, angle_radians, axes='sxyz')
            else:
                raise ValueError("Invalid axis for orientation bias")
            current_quat = [
                sensor_data.orientation.w,
                sensor_data.orientation.x,
                sensor_data.orientation.y,
                sensor_data.orientation.z,
            ]
            current_rot = quat2mat(current_quat)
            new_rot = current_rot @ bias_rot
            new_quat = mat2quat(new_rot)
            sensor_data.orientation.w = new_quat[0]
            sensor_data.orientation.x = new_quat[1]
            sensor_data.orientation.y = new_quat[2]
            sensor_data.orientation.z = new_quat[3]
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying orientation bias: {e}")
            return sensor_data

    def _apply_gyroscope_noise(self, sensor_data, fault):
        """
        Add Gaussian noise to IMU gyroscope (angular velocity).
        """
        try:
            noise_stddev = fault.get('parameters', {}).get('noise_stddev', {
                "x": 0.01,
                "y": 0.01,
                "z": 0.01
            })
            sensor_data.angular_velocity.x += np.random.normal(0, noise_stddev.get("x", 0.01))
            sensor_data.angular_velocity.y += np.random.normal(0, noise_stddev.get("y", 0.01))
            sensor_data.angular_velocity.z += np.random.normal(0, noise_stddev.get("z", 0.01))
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying gyroscope noise: {e}")
            return sensor_data

    def _apply_accelerometer_noise(self, sensor_data, fault):
        """
        Add Gaussian noise to IMU accelerometer (linear acceleration).
        """
        try:
            noise_stddev = fault.get('parameters', {}).get('noise_stddev', {
                "x": 0.01,
                "y": 0.01,
                "z": 0.01
            })
            sensor_data.linear_acceleration.x += np.random.normal(0, noise_stddev.get("x", 0.01))
            sensor_data.linear_acceleration.y += np.random.normal(0, noise_stddev.get("y", 0.01))
            sensor_data.linear_acceleration.z += np.random.normal(0, noise_stddev.get("z", 0.01))
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying accelerometer noise: {e}")
            return sensor_data

    def _apply_gyroscope_bias(self, sensor_data, fault):
        """
        Add constant bias to IMU gyroscope (angular velocity).
        """
        try:
            bias = fault.get('parameters', {}).get('bias', {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            })
            sensor_data.angular_velocity.x += bias.get("x", 0.0)
            sensor_data.angular_velocity.y += bias.get("y", 0.0)
            sensor_data.angular_velocity.z += bias.get("z", 0.0)
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying gyroscope bias: {e}")
            return sensor_data

    def _apply_accelerometer_bias(self, sensor_data, fault):
        """
        Add constant bias to IMU accelerometer (linear acceleration).
        """
        try:
            bias = fault.get('parameters', {}).get('bias', {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            })
            sensor_data.linear_acceleration.x += bias.get("x", 0.0)
            sensor_data.linear_acceleration.y += bias.get("y", 0.0)
            sensor_data.linear_acceleration.z += bias.get("z", 0.0)
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying accelerometer bias: {e}")
            return sensor_data