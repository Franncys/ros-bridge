from carla_ros_bridge.FaultInjector.FaultInjector import FaultInjector
import cv2
import numpy as np

class RGBCameraFaultInjector(FaultInjector):
    def __init__(self, config_file):
        super().__init__(config_file, "RGBCamera")

    def apply_faults(self, sensor_data):
        """
        Apply RGB camera-specific faults to the sensor data.
        """
        for fault in self.active_faults:
            if fault['failure_type'] == 'blur':
                sensor_data = self._apply_blur(sensor_data, fault)
            elif fault['failure_type'] == 'noise':
                sensor_data = self._apply_noise(sensor_data, fault)
            elif fault['failure_type'] == 'dropout':
                sensor_data = self._apply_dropout(sensor_data, fault)
            # Add more RGB camera-specific fault types as needed
        return sensor_data

    def _apply_blur(self, sensor_data, fault):
        """
        Apply a blur effect to the RGB camera image.
        """
        blur_kernel = fault.get('parameters', {}).get('blur_kernel', 5)
        image = sensor_data['data']  # Assuming sensor_data['data'] contains the image as a numpy array
        blurred_image = cv2.GaussianBlur(image, (blur_kernel, blur_kernel), 0)
        sensor_data['data'] = blurred_image
        return sensor_data