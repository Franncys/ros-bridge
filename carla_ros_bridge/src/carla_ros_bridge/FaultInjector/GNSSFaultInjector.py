from carla_ros_bridge.FaultInjector.FaultInjector import FaultInjector

class GNSSFaultInjector(FaultInjector):
    def __init__(self, config_file):
        super().__init__(config_file, "GNSSSensor")

    def apply_faults(self, sensor_data):
        """
        Apply GNSS-specific faults to the sensor data.
        """
        try:
            for fault in self.active_faults:
                self.logger.info(f"Applying fault: {fault['name']} with parameters: {fault['parameters']}")
                if fault['failure_type'] == 'bias':
                    sensor_data = self._apply_bias(sensor_data, fault)
                elif fault['failure_type'] == 'signal_loss':
                    sensor_data = self._apply_signal_loss(sensor_data, fault)
                # Add more GNSS-specific fault types as needed
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying faults to GNSS data: {e}")
            return sensor_data

    def _apply_bias(self, sensor_data, fault):
        """
        Apply a positional bias to the GNSS data.
        """
        try:
            bias = fault.get('parameters', {}).get('bias', {"latitude": 0.0, "longitude": 0.0, "altitude": 0.0})
            sensor_data['latitude'] += bias['latitude']
            sensor_data['longitude'] += bias['longitude']
            sensor_data['altitude'] += bias['altitude']
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying bias: {e}")
            return sensor_data

    def _apply_signal_loss(self, sensor_data, fault):
        """
        Simulate GNSS signal loss by dropping the data.
        """
        return None if fault.get('parameters', {}).get('signal_loss', True) else sensor_data