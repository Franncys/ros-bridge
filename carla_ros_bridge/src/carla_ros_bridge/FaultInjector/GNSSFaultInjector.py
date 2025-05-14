from carla_ros_bridge.FaultInjector.FaultInjector import FaultInjector

class GNSSFaultInjector(FaultInjector):
    def __init__(self, config_file=None):
        super().__init__("GNSSSensor", config_file)

    def apply_faults(self, sensor_data):
        """
        Apply GNSS-specific faults to the sensor data.
        """
        try:
            for active_fault in self.active_faults:
                fault = active_fault["fault"] 
                # Log sensor data before applying faults
                #self.logger.info("GNSS Sensor data before applying faults: %s", sensor_data)
                #self.logger.info(f"Applying fault: {fault['name']} with parameters: {fault['parameters']}")
                if fault['name'] == 'bias':
                    sensor_data = self._apply_bias(sensor_data, fault)
                elif fault['name'] == 'signal_loss':
                    sensor_data = self._apply_signal_loss(sensor_data, fault)
                elif fault['name'] == 'zero_value':
                    sensor_data = self._apply_zero_value(sensor_data, fault)
                # Add more GNSS-specific fault types as needed
            
            # Log sensor data after applying faults
            #self.logger.info("GNSS Sensor data after applying faults: %s", sensor_data)

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
    
    def _apply_zero_value(self, sensor_data, fault):
        """
        Simulate a "0 value" fault by setting GNSS data to zero.
        """
        try:
            sensor_data['latitude'] = 0.0
            sensor_data['longitude'] = 0.0
            sensor_data['altitude'] = 0.0
            return sensor_data
        except Exception as e:
            self.logger.error(f"Error applying zero value fault: {e}")
            return sensor_data
