#!usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla imu sensor
"""

from transforms3d.euler import euler2quat

import carla_common.transforms as trans

from carla_ros_bridge.sensor import Sensor

from carla_ros_bridge.FaultInjector.Tools import has_fault_for_sensor
from sensor_msgs.msg import Imu

from carla_ros_bridge.FaultInjector.IMUFaultInjector import IMUFaultInjector

class ImuSensor(Sensor):

    """
    Actor implementation details for imu sensor
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode, frame_id, fault_config_file=None):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor : carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(ImuSensor, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode)

        # Initialize the IMUFaultInjector only if faults exist for this sensor
        #if fault_config_file and has_fault_for_sensor(fault_config_file, "IMUSensor"):
        # We now load the injection file dynamically
        self.fault_injector = IMUFaultInjector()
        #else:
        #self.fault_injector = None
            
        self.imu_publisher = node.new_publisher(Imu, self.get_topic_prefix(), qos_profile=10)
        self.listen()

        self._frame_id = frame_id

    def destroy(self):
        super(ImuSensor, self).destroy()
        self.node.destroy_publisher(self.imu_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_imu_measurement):
        """
        Function to transform a received imu measurement into a ROS Imu message

        :param carla_imu_measurement: carla imu measurement object
        :type carla_imu_measurement: carla.IMUMeasurement
        """
        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(frame_id=self._frame_id, timestamp=carla_imu_measurement.timestamp)

        # Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
        # Here, these measurements are converted to the right-handed ROS convention
        #  (X forward, Y left, Z up).
        imu_msg.angular_velocity.x = -carla_imu_measurement.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.angular_velocity.z = -carla_imu_measurement.gyroscope.z

        imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
        imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        roll, pitch, yaw = trans.carla_rotation_to_RPY(carla_imu_measurement.transform.rotation)
        quat = euler2quat(roll, pitch, yaw)
        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        # Apply fault injection if enabled
        if self.fault_injector and self.fault_injector.skip_message == True:
            print("Skipping imu message due to fault injection.")
            return

        # Apply fault injection
        if self.fault_injector:
            imu_msg = self.fault_injector.apply_faults(imu_msg)

        self.imu_publisher.publish(imu_msg)