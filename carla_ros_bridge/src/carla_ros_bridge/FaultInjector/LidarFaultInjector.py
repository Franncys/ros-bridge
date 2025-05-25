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
                    sensor_data = self._apply_distance_bias(sensor_data, fault)
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
            
            if point_cloud.size == 0:
                return sensor_data

            # Optionally filter ego vehicle points
            filter_ego = fault.get('parameters', {}).get('filter_ego_vehicle', True)
            if filter_ego:
                point_cloud = self.filter_ego_vehicle_points(point_cloud, fault)

            
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
    
    def _apply_distance_bias(self, sensor_data, fault):
        """
        Add a fixed distance (e.g., 10 meters) outward from the origin to each LiDAR point,
        but do NOT apply to points at (0,0,0).
        """
        try:
            self.logger.info("Applying distance bias to LiDAR points.")

            points = np.asarray(sensor_data['points'], dtype=np.float32)
            if points.size == 0:
                return sensor_data
            
            #points = self.filter_nearby_points(points, min_radius=1.8)
            filter_ego = fault.get('parameters', {}).get('filter_ego_vehicle', True)
            if filter_ego:
                points = self.filter_ego_vehicle_points(points, fault)
            
            bias_distance = fault.get('parameters', {}).get('bias_percent', 0.5)

            xyz = points[:, :3]
            norms = np.linalg.norm(xyz, axis=1, keepdims=True)
            # Mask for points not at the origin
            mask = (norms[:, 0] != 0)
            directions = np.zeros_like(xyz)
            directions[mask] = xyz[mask] / norms[mask]

            # Only move points not at the origin
            xyz[mask] = xyz[mask] + directions[mask] * bias_distance

            points[:, :3] = xyz
            sensor_data['points'] = points
            self.logger.info("Applied distance bias to %d points.", mask.sum())
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying distance bias: {e}")
            return sensor_data
        
    def filter_nearby_points(self, points, min_radius=1.5):
        """
        Remove points within min_radius of the origin (0,0,0).
        points: np.ndarray of shape (N, 4) or (N, 5)
        Returns: filtered points (same shape)
        """
        xyz = points[:, :3]
        dists = np.linalg.norm(xyz, axis=1)
        mask = dists > min_radius
        return points[mask]
    
    def filter_ego_vehicle_points(self, points, fault=None):
        """
        Remove points inside the ego vehicle bounding box using vehicle parameters.
        """
        # Vehicle dimensions gotten from config on Autoware
        # length = 2.544 + 1.12 + 0.82  # 4.484
        # width = 1.45 + 0.18 + 0.18    # 1.81
        # height = 2.40
        # Get vehicle box from fault parametsers, or use defaults
        box = fault.get('parameters', {}).get('vehicle_box', {})
        length = box.get('length', 4.484)
        width = box.get('width', 1.81)
        height = box.get('height', 2.40)

        z_offset = box.get('z_offset', 0.0)

        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        mask = (
            (np.abs(x) > length / 2) |
            (np.abs(y) > width / 2) |
            (np.abs(z - z_offset) > height / 2)
        )
        return points[mask]

    def _apply_percentage_distance_bias(self, sensor_data, fault):
        """
        Add a percentage bias to the distance of each LiDAR point from the origin.
        For example, a 10% bias moves a point at 10m to 11m.
        """
        try:
            self.logger.info("Applying percentage distance bias to LiDAR points.")

            points = np.asarray(sensor_data['points'], dtype=np.float32)
            if points.size == 0:
                return sensor_data

            # Optionally filter ego vehicle points
            filter_ego = fault.get('parameters', {}).get('filter_ego_vehicle', True)
            if filter_ego:
                points = self.filter_ego_vehicle_points(points, fault)

            bias_percent = fault.get('parameters', {}).get('bias_percent', 0.0)
            scale = 1.0 + (bias_percent / 100.0)

            xyz = points[:, :3]
            norms = np.linalg.norm(xyz, axis=1, keepdims=True)
            # Mask for points not at the origin
            mask = (norms[:, 0] != 0)
            directions = np.zeros_like(xyz)
            directions[mask] = xyz[mask] / norms[mask]

            # Only move points not at the origin
            xyz[mask] = directions[mask] * (norms[mask] * scale)

            points[:, :3] = xyz
            sensor_data['points'] = points
            self.logger.info("Applied percentage distance bias (%.2f%%) to %d points.", bias_percent, mask.sum())
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying percentage distance bias: {e}")
            return sensor_data
