from carla_ros_bridge.FaultInjector.FaultInjector import FaultInjector
import numpy as np

class LidarFaultInjector(FaultInjector):
    def __init__(self, config_file=None):
        super().__init__("LidarSensor", config_file,)

    def apply_faults(self, sensor_data):
        """
        Apply Lidar-specific faults to the sensor data.
        """
        try:
            for active_fault in self.active_faults:
                fault = active_fault["fault"] 
                # Log sensor data before applying faults
                self.logger.info("Lidar Sensor data before applying faults: %s", sensor_data)
                
                self.logger.info(f"Applying fault: {fault['name']} with parameters: {fault['parameters']}")
                
                if fault['name'] == 'noise':
                    sensor_data = self._apply_noise(sensor_data, fault)
                elif fault['name'] == 'dropout':
                    sensor_data = self._apply_dropout(sensor_data, fault)
                elif fault['name'] == 'zero_value':
                    sensor_data = self._apply_zero_value(sensor_data, fault)
                # Add more Lidar-specific fault types as needed
            
            # Log sensor data after applying faults
            self.logger.info("Lidar Sensor data after applying faults: %s", sensor_data)

            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying faults to Lidar data: {e}")
            return sensor_data

    def _apply_noise(self, sensor_data, fault):
        """
        Add noise to the Lidar point cloud data.
        """
        try:
            if 'points' not in sensor_data or not isinstance(sensor_data['points'], np.ndarray):
                raise ValueError("Sensor data is missing point cloud information or is not a NumPy array.")
            noise_level = fault.get('parameters', {}).get('noise_level', 0.1)
            point_cloud = np.array(sensor_data['points'])  # Assuming sensor_data['points'] is a numpy array
            noise = np.random.normal(0, noise_level, point_cloud.shape)
            sensor_data['points'] = point_cloud + noise
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying noise: {e}")
            return sensor_data

    def _apply_dropout(self, sensor_data, fault):
        """
        Simulate data dropout by removing a percentage of points.
        """
        try:
            if 'points' not in sensor_data or not isinstance(sensor_data['points'], np.ndarray):
                raise ValueError("Sensor data is missing point cloud information or is not a NumPy array.")
            dropout_rate = fault.get('parameters', {}).get('dropout_rate', 0.1)
            point_cloud = np.array(sensor_data['points'])
            mask = np.random.rand(point_cloud.shape[0]) > dropout_rate
            sensor_data['points'] = point_cloud[mask]
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying dropout: {e}")
            return sensor_data
        
    def _apply_zero_value(self, sensor_data, fault):
        """
        Simulate a "0 value" fault by clearing the LIDAR point cloud data.
        """
        try:
            self.logger.info("Applying zero value fault to Lidar data.")
            self.logger.info("Sensor data before applying zero value fault: %s", sensor_data)
            sensor_data['points'] = np.zeros_like(sensor_data['points'])
            self.logger.info("Lidar Sensor data after applying zero value fault: %s", sensor_data)
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying zero value fault: {e}")
            return sensor_data