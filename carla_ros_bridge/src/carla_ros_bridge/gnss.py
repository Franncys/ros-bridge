#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla gnsss
"""

from carla_ros_bridge.sensor import Sensor

from carla_ros_bridge.FaultInjector.GNSSFaultInjector import GNSSFaultInjector
from carla_ros_bridge.FaultInjector.Tools import has_fault_for_sensor
from sensor_msgs.msg import NavSatFix

from carla_ros_bridge.FaultInjector.gnss_data import GNSSData


class Gnss(Sensor):

    """
    Actor implementation details for gnss sensor
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode, frame_id):#, fault_config_file=None):
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
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Gnss, self).__init__(uid=uid,
                                   name=name,
                                   parent=parent,
                                   relative_spawn_pose=relative_spawn_pose,
                                   node=node,
                                   carla_actor=carla_actor,
                                   synchronous_mode=synchronous_mode)

        # Initialize the GNSSFaultInjector only if faults exist for this sensor
        # if fault_config_file and has_fault_for_sensor(fault_config_file, "GNSSSensor"):
        #     self.fault_injector = GNSSFaultInjector(fault_config_file)
        # else:
        #     self.fault_injector = None
        self.fault_injector = GNSSFaultInjector()

        self.gnss_publisher = node.new_publisher(NavSatFix,
                                                 self.get_topic_prefix(),
                                                 qos_profile=10)
        self.listen()

        self._frame_id = frame_id

    def destroy(self):
        super(Gnss, self).destroy()
        self.node.destroy_publisher(self.gnss_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_gnss_measurement):
        """
        Function to transform a received gnss event into a ROS NavSatFix message

        :param carla_gnss_measurement: carla gnss measurement object
        :type carla_gnss_measurement: carla.GnssMeasurement
        """
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header = self.get_msg_header(frame_id=self._frame_id, timestamp=carla_gnss_measurement.timestamp)
        navsatfix_msg.latitude = carla_gnss_measurement.latitude
        navsatfix_msg.longitude = carla_gnss_measurement.longitude
        navsatfix_msg.altitude = carla_gnss_measurement.altitude

        # Apply fault injection if enabled
        if self.fault_injector and self.fault_injector.skip_message == True:
            print("Skipping GNSS message due to fault injection.")
            return

        # # Apply fault injection if enabled
        if self.fault_injector:
            navsatfix_msg = self.fault_injector.apply_faults(navsatfix_msg)

        self.gnss_publisher.publish(navsatfix_msg)

        # Extract GNSS data (latitude, longitude, altitude) from the sensor data
        current_location = {
            "latitude": carla_gnss_measurement.latitude,
            "longitude": carla_gnss_measurement.longitude,
            "altitude": carla_gnss_measurement.altitude
        }

        # Update the shared GNSSData class
        GNSSData.update_location(current_location)
