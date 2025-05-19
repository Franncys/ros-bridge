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
                #self.logger.info("Lidar Sensor data before applying faults: %s", sensor_data)
                
                self.logger.info(f"Applying fault: {fault['name']} with parameters: {fault['parameters']}")
                
                if fault['name'] == 'noise':
                    sensor_data = self._apply_noise(sensor_data, fault)
                # elif fault['name'] == 'dropout':
                #     sensor_data = self._apply_dropout(sensor_data, fault)
                elif fault['name'] == 'zero_value':
                    sensor_data = self._apply_zero_value(sensor_data, fault)
                elif fault['name'] == 'percentage_bias':
                    sensor_data = self._apply_percentage_bias(sensor_data, fault)
                # Add more Lidar-specific fault types as needed
            
            # Log sensor data after applying faults
            #self.logger.info("Lidar Sensor data after applying faults: %s", sensor_data)

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
            noisy_points = point_cloud + noise
            # Ensure correct types: x, y, z, intensity = float32; ring = uint16
            if noisy_points.shape[1] == 5:
                noisy_points[:, 0:4] = noisy_points[:, 0:4].astype(np.float32)
                noisy_points[:, 4] = noisy_points[:, 4].astype(np.uint16)
            sensor_data['points'] = noisy_points
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
            #self.logger.info("Sensor data before applying zero value fault: %s", sensor_data)
            sensor_data['points'] = np.zeros_like(sensor_data['points'])
            #self.logger.info("Lidar Sensor data after applying zero value fault: %s", sensor_data)
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying zero value fault: {e}")
            return sensor_data
    
    # def _apply_percentage_bias(self, sensor_data, fault):
    #     """
    #     Apply a percentage bias to the distance of each LiDAR point.
    #     """
    #     try:
    #         self.logger.info("Applying percentage bias fault to Lidar data.")
    #         bias_percent = fault.get('parameters', {}).get('bias_percent', 0)
    #         if bias_percent == 0:
    #             return sensor_data
    #         points = np.array(sensor_data['points'], dtype=np.float32)
    #         # Compute distances from origin
    #         distances = np.linalg.norm(points[:, :3], axis=1)
    #         # Scale distances by (1 + bias_percent/100)
    #         scale = 1 + bias_percent / 100.0
    #         new_distances = distances * scale
    #         # Avoid division by zero
    #         with np.errstate(divide='ignore', invalid='ignore'):
    #             factors = np.where(distances != 0, new_distances / distances, 1.0)
    #         # Scale x, y, z
    #         points[:, 0] *= factors
    #         points[:, 1] *= factors
    #         points[:, 2] *= factors
    #         # Restore correct types: x, y, z, intensity = float32; ring = uint16
    #         #log shape
    #         self.logger.info(f"Points shape: {points.shape}")
    #         if points.shape[1] == 5:
    #             points[:, 0:4] = points[:, 0:4].astype(np.float32)
    #             points[:, 4] = points[:, 4].astype(np.uint16)
    #         sensor_data['points'] = points
    #         self.logger.info("Lidar Sensor data after applying percentage bias fault: %s", sensor_data)
    #         return sensor_data
    #     except Exception as e:
    #         self.logger.error(f"Error applying percentage bias: {e}")
    #         return sensor_data
    
    #def _apply_percentage_bias(self, sensor_data, fault):
    #    """
    #    Apply a percentage bias to the distance of each LiDAR point.
    #    """
    #    try:
    #        self.logger.info("Applying percentage bias fault to Lidar data.")
    #        bias_percent = fault.get('parameters', {}).get('bias_percent', 0)
    #        if bias_percent == 0:
    #            return sensor_data

    #        # Ensure points is a proper float32 NumPy array
    #        points = np.array(sensor_data['points'], dtype=np.float32)
    #        if points.shape[0] == 0:
    #            return sensor_data

    #        # Calculate scaling factor
    #        scale = 1 + bias_percent / 100.0

    #        # Scale only x, y, z
    #        xyz = points[:, :3]
    #        # Compute distances and directions
    #        norms = np.linalg.norm(xyz, axis=1, keepdims=True)
    #        # Avoid division by zero
    #        directions = np.divide(xyz, norms, out=np.zeros_like(xyz), where=norms!=0)
    #        new_xyz = xyz + directions * (norms * (scale - 1))

    #        # Build new points array
    #        new_points = np.copy(points)
    #        new_points[:, :3] = new_xyz

    #        # Ensure correct types: x, y, z, intensity = float32; ring = uint16
    #        if new_points.shape[1] == 5:
    #            new_points[:, 0:4] = new_points[:, 0:4].astype(np.float32)
    #            new_points[:, 4] = new_points[:, 4].astype(np.uint16)

    #        sensor_data['points'] = new_points
    #        self.logger.info("Lidar Sensor data after applying percentage bias fault: %s", sensor_data)
    #        return sensor_data
    #    except Exception as e:
    #        self.logger.error(f"Error applying percentage bias: {e}")
    #        return sensor_data
        
    def _apply_percentage_bias(self, sensor_data, fault):
        """
        Apply a percentage bias to the distance of each LiDAR point.
        """
        try:
            self.logger.info("Applying percentage bias fault to Lidar data.")
            bias_percent = fault.get('parameters', {}).get('bias_percent', 0)
            if bias_percent == 0:
                return sensor_data

            points = np.asarray(sensor_data['points'])
            if points.shape[0] == 0:
                return sensor_data

            # Separate colsssssumn
            xyz = points[:, :3].astype(np.float32)
            intensity = points[:, 3].astype(np.float32).reshape(-1, 1)
            if points.shape[1] > 4:
                ring = points[:, 4].astype(np.uint16).reshape(-1, 1)
            else:
                ring = None

            # Calculate scaling factors
            scale = 1 + bias_percent / 100.0

            # Compute distances and directions
            norms = np.linalg.norm(xyz, axis=1, keepdims=True)
            directions = np.divide(xyz, norms, out=np.zeros_like(xyz), where=norms != 0)
            new_xyz = xyz + directions * (norms * (scale - 1))

            # Recombine all columns with correct types
            if ring is not None:
                new_points = np.hstack((new_xyz.astype(np.float32), intensity.astype(np.float32), ring.astype(np.uint16)))
            else:
                new_points = np.hstack((new_xyz.astype(np.float32), intensity.astype(np.float32)))

            sensor_data['points'] = new_points
            self.logger.info("Lidar Sensor data after applying percentage bias fault: %s", sensor_data)
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying percentage bias: {e}")
            return sensor_data