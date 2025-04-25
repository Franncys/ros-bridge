from carla_ros_bridge.FaultInjector.FaultInjector import FaultInjector
import numpy as np

class LidarFaultInjector(FaultInjector):
    def __init__(self, config_file):
        super().__init__(config_file, "LidarSensor")

    def apply_faults(self, sensor_data):
        """
        Apply Lidar-specific faults to the sensor data.
        """
        for fault in self.active_faults:
            if fault['failure_type'] == 'noise':
                sensor_data = self._apply_noise(sensor_data, fault)
            elif fault['failure_type'] == 'dropout':
                sensor_data = self._apply_dropout(sensor_data, fault)
            # Add more Lidar-specific fault types as needed
        return sensor_data

    def _apply_noise(self, sensor_data, fault):
        """
        Add noise to the Lidar point cloud data.
        """
        if 'points' not in sensor_data or not isinstance(sensor_data['points'], np.ndarray):
            raise ValueError("Sensor data is missing point cloud information or is not a NumPy array.")
        noise_level = fault.get('parameters', {}).get('noise_level', 0.1)
        point_cloud = np.array(sensor_data['points'])  # Assuming sensor_data['points'] is a numpy array
        noise = np.random.normal(0, noise_level, point_cloud.shape)
        sensor_data['points'] = point_cloud + noise
        return sensor_data

    def _apply_dropout(self, sensor_data, fault):
        """
        Simulate data dropout by removing a percentage of points.
        """
        if 'points' not in sensor_data or not isinstance(sensor_data['points'], np.ndarray):
            raise ValueError("Sensor data is missing point cloud information or is not a NumPy array.")
        dropout_rate = fault.get('parameters', {}).get('dropout_rate', 0.1)
        point_cloud = np.array(sensor_data['points'])
        mask = np.random.rand(point_cloud.shape[0]) > dropout_rate
        sensor_data['points'] = point_cloud[mask]
        return sensor_data