#!/usr/bin/env python

#
# Copyright (c) 2018, Willow Garage, Inc.
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla lidars
"""

import numpy
import json
from datetime import datetime
import logging

from carla_ros_bridge.sensor import Sensor, create_cloud

from carla_ros_bridge.FaultInjector.LidarFaultInjector import LidarFaultInjector
from carla_ros_bridge.FaultInjector.Tools import has_fault_for_sensor
from sensor_msgs.msg import PointCloud2, PointField
from rclpy import qos

logger = logging.getLogger(__name__)

class Lidar(Sensor):

    """
    Actor implementation details for lidars
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
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Lidar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)
        
        # Initialize the LidarFaultInjector only if faults exist for this sensor
        if fault_config_file and has_fault_for_sensor(fault_config_file, "LidarSensor"):
            self.fault_injector = LidarFaultInjector(fault_config_file)
        else:
            self.fault_injector = None

        self.lidar_publisher = node.new_publisher(PointCloud2,
                                                  self.get_topic_prefix(),
                                                  qos_profile=qos.qos_profile_sensor_data)
        self.listen()
        self.channels = int(self.carla_actor.attributes.get('channels'))
        self._frame_id = frame_id

    def destroy(self):
        super(Lidar, self).destroy()
        self.node.destroy_publisher(self.lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        header = self.get_msg_header(frame_id=self._frame_id , timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1)
        ]

        lidar_data = numpy.fromstring(
            bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        
        ring = numpy.empty((0,1), object)
        
        for i in range(self.channels):
            current_ring_points_count = carla_lidar_measurement.get_point_count(i)
            ring = numpy.vstack((
                ring,
                numpy.full((current_ring_points_count, 1), i)))
        
        lidar_data = numpy.hstack((lidar_data, ring))
        
        # Apply fault injection if enabled
        if self.fault_injector:
            lidar_data = self.fault_injector.apply_faults({"points": lidar_data})["points"]

        # we take the opposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data[:, 1] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data)
        self.lidar_publisher.publish(point_cloud_msg)

        # Log the LiDAR data
        self.log_sensor_data(carla_lidar_measurement)

    def log_sensor_data(self, carla_lidar_measurement):
        """
        Log LiDAR-specific data.

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        log_entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "sensor_type": "Lidar",
            "sensor_id": self.uid,
            "frame_id": carla_lidar_measurement.frame,
            "point_count": len(carla_lidar_measurement.raw_data) // 16,  # Assuming 16 bytes per point
        }
        logger.info(json.dumps(log_entry))


class SemanticLidar(Sensor):

    """
    Actor implementation details for semantic lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(SemanticLidar, self).__init__(uid=uid,
                                            name=name,
                                            parent=parent,
                                            relative_spawn_pose=relative_spawn_pose,
                                            node=node,
                                            carla_actor=carla_actor,
                                            synchronous_mode=synchronous_mode)

        self.semantic_lidar_publisher = node.new_publisher(
            PointCloud2,
            self.get_topic_prefix(),
            qos_profile=10)
        self.listen()

    def destroy(self):
        super(SemanticLidar, self).destroy()
        self.node.destroy_publisher(self.semantic_lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform a received semantic lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla semantic lidar measurement object
        :type carla_lidar_measurement: carla.SemanticLidarMeasurement
        """
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='CosAngle', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ObjIdx', offset=16, datatype=PointField.UINT32, count=1),
            PointField(name='ObjTag', offset=20, datatype=PointField.UINT32, count=1)
        ]

        lidar_data = numpy.fromstring(bytes(carla_lidar_measurement.raw_data),
                                      dtype=numpy.dtype([
                                          ('x', numpy.float32),
                                          ('y', numpy.float32),
                                          ('z', numpy.float32),
                                          ('CosAngle', numpy.float32),
                                          ('ObjIdx', numpy.uint32),
                                          ('ObjTag', numpy.uint32)
                                      ]))

        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data['y'] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data.tolist())
        self.semantic_lidar_publisher.publish(point_cloud_msg)

        # Log the Semantic LiDAR data
        self.log_sensor_data(carla_lidar_measurement)

    def log_sensor_data(self, carla_lidar_measurement):
        """
        Log Semantic LiDAR-specific data.

        :param carla_lidar_measurement: carla semantic lidar measurement object
        :type carla_lidar_measurement: carla.SemanticLidarMeasurement
        """
        log_entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "sensor_type": "SemanticLidar",
            "sensor_id": self.uid,
            "frame_id": carla_lidar_measurement.frame,
            "object_count": len(carla_lidar_measurement.raw_data) // 24,  # Assuming 24 bytes per point
        }
        logger.info(json.dumps(log_entry))
