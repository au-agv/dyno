#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( D ) ( Y ) ( N ) ( O )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
#############################################################################

#  MIT License
#
#  DYNO: Ground Vehicle Dynamics Validation Toolkit
#  Copyright (c) 2024 Dario Sirangelo (dev@dariosirangelo.me).
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to
#  deal in the Software without restriction, including without limitation the
#  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
#  sell copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
#  IN THE SOFTWARE.

import logging
import re

from datetime import datetime
from typing import (
    Any,
    Callable,
    Dict,
    Generator,
    List,
    Optional,
    Sequence,
    Tuple,
)
from collections import defaultdict

import numpy as np
import pandas as pd
import rosbags.highlevel
import rosbags.typesys
import tables
import yaml

from pathlib import Path

import pydyno.tooling

from pydyno.processing.core import Processor


class BagDataProcessor(Processor):

    def __init__(self):

        pass

    @staticmethod
    def _load_bag(path: Path):

        logging.info("Reading bag '%s' ...", path.stem)

        bag = rosbags.highlevel.AnyReader(
            [path.parent],
            default_typestore=rosbags.typesys.get_typestore(
                rosbags.typesys.Stores.ROS2_HUMBLE
            ),
        )
        bag.open()

        return bag

    @staticmethod
    def _get_connections(bag):

        return [connection for connection in bag.connections]

    @staticmethod
    def _list_topics(connections) -> None:
        [
            logging.info(
                f"Found topic '{connection.topic}' '{connection.msgtype}'"
            )
            for connection in connections
        ]

    @staticmethod
    def _get_messages(bag):
        return bag.messages(connections=bag.connections)

    @staticmethod
    def _get_first_timestamp(messages) -> None:

        timestamp = min([timestamp for (_, timestamp, _) in messages])
        logging.info(
            "Bag initial timestamp: %s",
            datetime.fromtimestamp(timestamp / 1.0e9).strftime(
                "%Y-%m-%d %H:%M:%S"
            ),
        )
        return timestamp

    @staticmethod
    def _get_last_timestamp(messages) -> None:

        timestamp = max([timestamp for (_, timestamp, _) in messages])
        logging.info(
            "Bag final timestamp: %s",
            datetime.fromtimestamp(timestamp / 1.0e9).strftime(
                "%Y-%m-%d %H:%M:%S"
            ),
        )
        return timestamp

    @staticmethod
    def _parse_topic(bag, topic: str):

        timestamps = []
        datas = []

        for connection, timestamp, data in bag.messages(
            connections=[
                connection
                for connection in bag.connections
                if connection.topic == topic
            ]
        ):

            timestamps.append(timestamp)
            datas.append(bag.deserialize(data, connection.msgtype))
        return timestamps, datas

    def _get_bag(self, key):

        return self._bags[key]

    def _get_value(self, bag, topic, field):
        messages = self._parse_topic(bag, topic)[1]
        res = [
            pydyno.tooling.deep_getattr(message, field) for message in messages
        ]

        return np.array(res)

    def get_specific_timeseries_set(
        self, quantities: List[str], fields: Dict[str, Callable]
    ):
        # quantities = self.__class__._ensure_list_arg(quantities)

        dataframes = []

        for quantity in quantities:
            for key, bag in self._bags.items():

                add_to_dataframes: bool = True
                for mkey, function in fields.items():
                    if not function(self._get_value(bag, quantity, mkey)):
                        add_to_dataframes = False
                if not add_to_dataframes:
                    continue

                dataframes.append(
                    pd.DataFrame(
                        {
                            "time": pydyno.tooling.deep_getattr(
                                database.root.data, "time"
                            ),
                            "value": self._parse_topic(bag, quantity, mkey),
                            "series": key,
                            "quantity": quantity,
                            # **self.get_all_metadata(database),
                        }
                    )
                )
        return pd.concat(dataframes, ignore_index=True)


class UniBagDataProcessor(BagDataProcessor):

    def __init__(self, bag_path: Path, database_path: Path):

        super().__init__()

        # Loading ROS bag.
        self._bag = self.__class__._load_bag(bag_path)

        # Conversion to HDF5 file.
        self._database = tables.open_file(
            database_path,
            mode="w",
            driver="H5FD_CORE",
            driver_core_backing_store=0,
        )
        self._compression = tables.Filters(complevel=0, complib=None)
        self._base_group = self._create_base_group()
        self._convert_to_hdf5(self._bag)

        # Conversion from YAML file
        parameters_group = self._create_group_and_parents(
            "parameters", base_group=self._base_group
        )

        with open(
            "/home/taskbjorn/repositories/github.com/au-agv/dyno/examples/postprocessor/metadata.yaml",
            "r",
        ) as file:
            self.serialize_dict_to_group(
                parameters_group,
                yaml.safe_load(file),
                filters=self._compression,
            )

    def _create_base_group(self) -> None:

        logging.info(
            'Creating base group under "%s"',
            self._HIERARCHICAL_DATABASE_ROS_PREFIX,
        )

        # Create the group hierarchy for the base group
        return self._create_group_and_parents(
            self._HIERARCHICAL_DATABASE_ROS_PREFIX
        )

    @staticmethod
    def _get_topics(
        bag: rosbags.highlevel.AnyReader,
    ) -> Dict[str, Tuple[int, Any]]:

        topics = defaultdict(list)

        for connection, timestamp, raw_data in bag.messages():
            message = bag.deserialize(raw_data, connection.msgtype)
            topics[connection.topic].append((timestamp, message))

        return topics

    def _convert_ros_time_to_timestamp(message):
        # ROS2 builtin_interfaces/Time
        return message.sec + message.nanosec * 1.0e-9

    def walk_msg(msg, prefix=""):
        if hasattr(msg, "__slots__"):
            for slot in msg.__slots__:
                val = getattr(msg, slot)

                # Convert ROS time immediately
                val = UniBagDataProcessor.ros_time_to_float(val)

                new_prefix = f"{prefix}/{slot}" if prefix else slot
                yield from UniBagDataProcessor.walk_msg(val, new_prefix)
        else:
            # Leaf
            yield prefix or "data", msg

    def _create_group_and_parents(
        self, path: str, base_group: Optional[str] = None
    ) -> None:
        """
        Create a group and all missing parents.
        Returns the final group.
        """
        if path == "/":
            return

        if base_group is None:
            current_group = self._database.root
        else:
            current_group = base_group

        for name in path.strip("/").split("/"):
            if name in current_group._v_children:
                current_group = current_group._v_children[name]
            else:
                logging.info(
                    'Creating new group "%s" under "%s" ...',
                    name,
                    current_group._v_pathname,
                )
                current_group = self._database.create_group(current_group, name)

        return current_group

    def _convert_topic_name(self, topic: str) -> str:
        # Create a topic group name using dot addressing.

        return topic.strip("/").replace("/", ".")

    def _create_topic_group(self, topic: str) -> None:

        if topic not in self._base_group:

            return self._create_group_and_parents(
                topic, base_group=self._base_group
            )

        return self._base_group[topic]

    def _create_earray(
        self,
        name: str,
        path: str,
        atom=tables.Float64Atom(),
        size: Optional[int] = None,
    ):

        logging.debug(
            'Creating new extendable array "%s" under "%s"',
            name,
            path,
        )

        if size is not None:
            shape = (0, size)
        else:
            shape = (0,)

        if name in path._v_children:
            return path._v_children[name]

        return self._database.create_earray(
            path,
            name,
            atom=atom,
            shape=shape,
            filters=self._compression,
        )

    def _get_nested_group(self, path, parent_group="/"):
        """
        Retrieve a nested group from a PyTables group safely.

        Parameters
        ----------
        parent_group : tables.Group
            The PyTables group to start from (can be root).
        path : str
            The nested path of the group relative to parent_group, e.g., "subgroup1/subgroup2".

        Returns
        -------
        tables.Group or None
            The nested group if it exists, otherwise None.
        """
        current = parent_group
        for name in path.strip("/").split("/"):
            current = current._v_children[name]
            if current is None or not isinstance(current, tables.Group):
                return None
        return current

    def _node_exists(self, parent, name):
        """
        Check if a node exists under parent (can be a group or root).
        """
        try:
            parent._v_file.get_node(parent, name)
            return True
        except tables.NoSuchNodeError:
            return False

    def _create_topic_timestamps(self, topic: str) -> None:

        # If the topic is nested under one or multiple namespace (e.g.
        # "namespace1/namespace2/topic_name"), we must first retrieve the
        # corresponding nested group in the HDF5 database.
        if len(topic.split("/")) > 1:
            nested_group = self._get_nested_group(
                topic, parent_group=self._base_group
            )
        else:
            nested_group = self._base_group[topic]

        if self._node_exists(nested_group, "timestamps"):
            logging.debug(
                'Timestamps earray already exists for topic "%s" under "%s"'
                " skipping ...",
                topic,
                self._base_group._v_pathname,
            )
            return

        self._create_earray("timestamps", nested_group)

    def _store_timestamp(self, timestamp, topic_group):

        self._create_earray("timestamps", path=topic_group).append([timestamp])

    # ------------------------------------------------------------------------ #
    # ROS message serialization/deserialization
    # ------------------------------------------------------------------------ #

    def _deserialize_message(self, message: Any, topic_group: tables.Group):

        if message.__msgtype__ not in self._DESERIALIZABLE_MESSAGE_TYPES.keys():
            logging.warning(
                'Skipping non-deserializable message type "%s" ...',
                message.__msgtype__,
            )
            return

        self._DESERIALIZABLE_MESSAGE_TYPES[message.__msgtype__](
            self, message, topic_group
        )

    def _deserialize_std_msgs_float64(
        self, message: Any, topic_group: tables.Group
    ):

        self._create_earray("data", topic_group).append([message.data])

    def _deserialize_builtin_interfaces_time(
        self, message: Any, topic_group: tables.Group
    ) -> None:

        self._create_earray("sec", topic_group).append([message.sec])
        self._create_earray("nanosec", topic_group).append([message.nanosec])

    def _deserialize_rosgraph_msgs_clock(
        self, message: Any, topic_group: tables.Group
    ) -> None:

        logging.warning("Skipping clock deserialization ...")
        pass

        self._create_group_and_parents("clock", topic_group)
        self._deserialize_builtin_interfaces_time(
            message.clock, self._create_group_and_parents("clock", topic_group)
        )

    def _deserialize_nav_msgs_odometry(self, message, topic_group):
        self._deserialize_std_msgs_header(
            message.header,
            self._create_group_and_parents("header", topic_group),
        )
        self._create_vlarray("child_frame_id", topic_group).append(
            message.child_frame_id.encode(encoding="utf-8")
        )
        self._deserialize_geometry_msgs_posewithcovariance(
            message.pose,
            self._create_group_and_parents("pose", topic_group),
        )
        self._deserialize_geometry_msgs_twistwithcovariance(
            message.twist,
            self._create_group_and_parents("twist", topic_group),
        )

    def _deserialize_geometry_msgs_posewithcovariance(
        self, message, topic_group
    ):

        self._deserialize_geometry_msgs_pose(
            message.pose,
            self._create_group_and_parents("pose", topic_group),
        )
        self._create_earray(
            "covariance", topic_group, size=len(message.covariance)
        ).append([message.covariance])

    def _deserialize_geometry_msgs_twistwithcovariance(
        self, message, topic_group
    ):

        self._deserialize_geometry_msgs_twist(
            message.twist,
            self._create_group_and_parents("twist", topic_group),
        )
        self._create_earray(
            "covariance", topic_group, size=len(message.covariance)
        ).append([message.covariance])

    def _deserialize_ackermann_msgs_ackermanndrivestamped(
        self, message: Any, topic_group: tables.Group
    ) -> None:

        self._deserialize_std_msgs_header(
            message.header,
            self._create_group_and_parents("header", topic_group),
        )
        self._deserialize_ackermann_msgs_ackermanndrive(
            message.drive, self._create_group_and_parents("drive", topic_group)
        )

    def _deserialize_ackermann_msgs_ackermanndrive(
        self, message: Any, topic_group: tables.Group
    ) -> None:

        self._create_earray("steering_angle", topic_group).append(
            [message.steering_angle]
        )
        self._create_earray("steering_angle_velocity", topic_group).append(
            [message.steering_angle_velocity]
        )
        self._create_earray("speed", topic_group).append([message.speed])
        self._create_earray("acceleration", topic_group).append(
            [message.acceleration]
        )
        self._create_earray("jerk", topic_group).append([message.jerk])

    def _deserialize_std_msgs_header(
        self, message: Any, topic_group: tables.Group
    ) -> None:

        self._deserialize_builtin_interfaces_time(
            message.stamp, self._create_group_and_parents("stamp", topic_group)
        )
        self._create_vlarray(
            "frame_id",
            topic_group,
            atom=tables.VLStringAtom(),
        ).append(message.frame_id.encode(encoding="utf-8"))

    def _create_vlarray(self, name: str, path: str, atom=tables.VLStringAtom()):

        logging.debug(
            'Creating new variable-length array "%s" under "%s"',
            name,
            path,
        )

        if name in path._v_children:
            return path._v_children[name]

        return self._database.create_vlarray(
            path,
            name,
            atom=atom,
            filters=self._compression,
        )

    def _get_string_atom(self, string: str) -> tables.StringAtom:

        return tables.StringAtom(itemsize=max(1, len(string.encode("utf-8"))))

    def _deserialize_geometry_msgs_transformstamped(
        self, message: Any, topic_group: tables.Group
    ) -> None:

        self._deserialize_std_msgs_header(
            message.header,
            self._create_group_and_parents("header", topic_group),
        )
        self._create_vlarray("child_frame_id", topic_group).append(
            message.child_frame_id.encode(encoding="utf-8")
        )
        self._deserialize_geometry_msgs_transform(
            message.transform,
            self._create_group_and_parents("transform", topic_group),
        )

    def _deserialize_geometry_msgs_transform(self, message, topic_group):

        self._deserialize_geometry_msgs_vector3(
            message.translation,
            self._create_group_and_parents("translation", topic_group),
        )
        self._deserialize_geometry_msgs_quaternion(
            message.rotation,
            self._create_group_and_parents("quaternion", topic_group),
        )

    def _deserialize_geometry_msgs_pose(self, message, topic_group):

        self._deserialize_geometry_msgs_point(
            message.position,
            self._create_group_and_parents("position", topic_group),
        )
        self._deserialize_geometry_msgs_quaternion(
            message.orientation,
            self._create_group_and_parents("orientation", topic_group),
        )

    def _deserialize_geometry_msgs_twist(self, message, topic_group):

        self._deserialize_geometry_msgs_vector3(
            message.linear,
            self._create_group_and_parents("linear", topic_group),
        )
        self._deserialize_geometry_msgs_vector3(
            message.angular,
            self._create_group_and_parents("angular", topic_group),
        )

    def _deserialize_geometry_msgs_vector3(self, message, topic_group):

        self._create_earray("x", topic_group).append([message.x])
        self._create_earray("y", topic_group).append([message.y])
        self._create_earray("z", topic_group).append([message.z])

    def _deserialize_geometry_msgs_point(self, message, topic_group):

        self._create_earray("x", topic_group).append([message.x])
        self._create_earray("y", topic_group).append([message.y])
        self._create_earray("z", topic_group).append([message.z])

    def _deserialize_geometry_msgs_quaternion(self, message, topic_group):

        self._create_earray("x", topic_group).append([message.x])
        self._create_earray("y", topic_group).append([message.y])
        self._create_earray("z", topic_group).append([message.z])
        self._create_earray("w", topic_group).append([message.w])

    def _deserialize_list(self, field_entries, topic_group, function):

        for index, entry in enumerate(field_entries):
            function(
                entry,
                self._create_group_and_parents(f"item{index:05d}", topic_group),
            )

    def _deserialize_nav_msgs_path(
        self, message: Any, topic_group: tables.Group
    ) -> None:
        self._deserialize_std_msgs_header(
            message.header,
            self._create_group_and_parents("header", topic_group),
        )
        self._deserialize_list(
            message.poses,
            self._create_group_and_parents("poses", topic_group),
            self._deserialize_geometry_msgs_posestamped,
        )

    def _deserialize_geometry_msgs_posestamped(self, message, topic_group):

        self._deserialize_std_msgs_header(
            message.header,
            self._create_group_and_parents("header", topic_group),
        )
        self._deserialize_geometry_msgs_pose(
            message.pose,
            self._create_group_and_parents("pose", topic_group),
        )

    def _deserialize_tf2_msgs_tfmessage(
        self, message: Any, topic_group: tables.Group
    ) -> None:

        logging.warning("Skipping transforms deserialization ...")
        pass

        self._deserialize_list(
            message.transforms,
            self._create_group_and_parents("transforms", topic_group),
            self._deserialize_geometry_msgs_transformstamped,
        )

    _ROS_PRIMITIVE_TYPES = (int, float, bool, str, bytes)

    _DESERIALIZABLE_MESSAGE_TYPES: dict = {
        "tf2_msgs/msg/TFMessage": _deserialize_tf2_msgs_tfmessage,
        "std_msgs/msg/Float64": _deserialize_std_msgs_float64,
        "rosgraph_msgs/msg/Clock": _deserialize_rosgraph_msgs_clock,
        "builtin_interfaces/msg/Time": _deserialize_builtin_interfaces_time,
        "std_msgs/msg/Header": _deserialize_std_msgs_header,
        "ackermann_msgs/msg/AckermannDriveStamped": (
            _deserialize_ackermann_msgs_ackermanndrivestamped
        ),
        "ackermann_msgs/msg/AckermannDrive": (
            _deserialize_ackermann_msgs_ackermanndrive
        ),
        "geometry_msgs/msg/Vector3": _deserialize_geometry_msgs_vector3,
        "geometry_msgs/msg/Quaternion": _deserialize_geometry_msgs_quaternion,
        "geometry_msgs/msg/Transform": _deserialize_geometry_msgs_transform,
        "geometry_msgs/msg/TransformStamped": (
            _deserialize_geometry_msgs_transformstamped
        ),
        "nav_msgs/msg/Odometry": _deserialize_nav_msgs_odometry,
        "geometry_msgs/msg/Point": _deserialize_geometry_msgs_point,
        "geometry_msgs/msg/TwistWithCovariance": (
            _deserialize_geometry_msgs_twistwithcovariance
        ),
        "geometry_msgs/msg/PoseWithCovariance": (
            _deserialize_geometry_msgs_posewithcovariance
        ),
        "geometry_msgs/msg/Twist": _deserialize_geometry_msgs_twist,
        "geometry_msgs/msg/Pose": _deserialize_geometry_msgs_pose,
        "geometry_msgs/msg/PoseStamped": _deserialize_geometry_msgs_posestamped,
        "nav_msgs/msg/Path": _deserialize_nav_msgs_path,
    }

    @staticmethod
    def _is_nested_message(object) -> bool:
        """
        Check if the object is a nested ROS message (has __slots__).
        """
        return hasattr(object, "__slots__")

    @classmethod
    def _is_list_of_messages(cls, object) -> bool:
        """
        Check if the object is a list of nested ROS messages.
        """
        return (
            isinstance(object, Sequence)
            and object
            and cls._is_nested_message(object[0])
        )

    def _has_subfields(message, field_name) -> bool:
        value = getattr(message, field_name, None)
        # Check if the value has __slots__ (i.e., is a nested ROS message)
        return hasattr(value, "__slots__")

    def sanitize_hdf5_name(name: str) -> str:
        """
        Replace characters not allowed in HDF5 node names with underscores.
        Allowed: alphanumerics, '_', '-', '.'
        """
        return re.sub(r"[^0-9a-zA-Z_\-\.]", "_", name)

    @classmethod
    def serialize_dict_to_group(cls, group, data, filters=None):
        h5 = group._v_file

        for key, value in data.items():
            key = cls.sanitize_hdf5_name(key)

            # Nested dict
            if isinstance(value, dict):
                subgroup = group._v_children.get(key)
                if subgroup is None:
                    subgroup = h5.create_group(group, key)
                cls.serialize_dict_to_group(subgroup, value, filters)

            # List
            elif isinstance(value, list):
                if not value:
                    continue

                if all(isinstance(v, (int, float, bool)) for v in value):
                    atom = (
                        tables.Float64Atom()
                        if any(isinstance(v, float) for v in value)
                        else tables.Int64Atom()
                    )
                    arr = group._v_children.get(key)
                    if arr is None:
                        arr = h5.create_earray(
                            group, key, atom, shape=(0,), filters=filters
                        )
                    arr.append(value)

                elif all(isinstance(v, str) for v in value):
                    maxlen = max(len(v) for v in value)
                    arr = group._v_children.get(key)
                    if arr is None:
                        arr = h5.create_carray(
                            group,
                            key,
                            tables.StringAtom(itemsize=maxlen),
                            shape=(len(value),),
                        )
                    arr[:] = [s.encode("utf-8") for s in value]

                else:
                    raise ValueError(
                        "Only homogeneous lists of numbers or strings are"
                        f" supported: {value}"
                    )

            # Primitive
            elif isinstance(value, (int, float, bool)):
                arr = group._v_children.get(key)
                if arr is None:
                    if isinstance(value, float):
                        atom = tables.Float64Atom()
                    elif isinstance(value, int):
                        atom = tables.Int64Atom()
                    elif isinstance(value, bool):
                        atom = tables.BoolAtom()
                    arr = h5.create_carray(group, key, atom, shape=(1,))
                arr[0] = value

            elif isinstance(value, str):
                arr = group._v_children.get(key)
                if arr is None:
                    arr = h5.create_carray(
                        group,
                        key,
                        tables.StringAtom(itemsize=len(value)),
                        shape=(1,),
                    )
                arr[0] = value.encode("utf-8")

            # Fallback
            else:
                s = str(value)
                arr = group._v_children.get(key)
                if arr is None:
                    arr = h5.create_carray(
                        group,
                        key,
                        tables.StringAtom(itemsize=len(s)),
                        shape=(1,),
                    )
                arr[0] = s.encode("utf-8")

    def _convert_to_hdf5(self, reader) -> None:

        for connection, timestamp, raw_data in reader.messages():
            if connection.msgtype in [
                "tf2_msgs/msg/TFMessage",
                "rosgraph_msgs/msg/Clock",
                "visualization_msgs/msg/MarkerArray",
                "navi_interfaces/msg/DwaStatus"
            ]:
                continue


            # topic = self._convert_topic_name(connection.topic)
            topic = connection.topic.lstrip("/")
            topic_group = self._create_topic_group(topic)
            self._create_topic_timestamps(topic)


            # Deserialize the message
            message = reader.deserialize(raw_data, connection.msgtype)

            self._store_timestamp(timestamp, topic_group)
            self._deserialize_message(message, topic_group)

    @staticmethod
    def _load_bag(path: Path):
        # Look for MCAP-formatted Rosbags under one-level of nesting relative to
        # the specified path.
        bag_files = list(path.glob("**/*.mcap"))
        if len(bag_files) < 1:
            logging.error(
                "No MCAP Rosbag files found under '%s'", path.as_posix()
            )
            raise FileExistsError("")
        elif len(bag_files) > 1:
            bag_files = [max(bag_files, key=lambda file: file.stat().st_mtime)]
            logging.warning(
                "More than one bag found, selecting the latest one (%s)...",
                bag_files[0].as_posix(),
            )

        # Parse and return the Rosbags in a dictionary.
        return BagDataProcessor._load_bag(bag_files[0])

    _HIERARCHICAL_DATABASE_ROS_PREFIX = "data/ros/topics"


class MultiBagDataProcessor(BagDataProcessor):

    def __init__(self, path: Path):

        super().__init__()

        self._bags = self._load_bags(path)
        self._connections = self.__class__._load_connections(self._bags)
        self._messages = self.__class__._load_messages(self._bags)

    @staticmethod
    def _load_messages(bags):
        messages = {}
        for key, bag in bags.items():
            messages = MultiBagDataProcessor._get_messages(bag)
        return messages

    @staticmethod
    def _load_connections(bags):
        connections = {}
        for key, bag in bags.items():
            connections[key] = MultiBagDataProcessor._get_connections(bag)
        return connections

    @staticmethod
    def _load_bags(path: Path):

        # Look for MCAP-formatted Rosbags under one-level of nesting relative to
        # the specified path.
        bag_files = list(path.glob("**/*.mcap"))
        if len(bag_files) < 1:
            logging.error(
                "No MCAP Rosbag files found under '%s'", path.as_posix()
            )
            raise FileExistsError("")

        # Parse and return the Rosbags in a dictionary.
        bags = {}
        for file in bag_files:
            bags[Processor._get_data_file_key(file)] = (
                BagDataProcessor._load_bag(file)
            )

        return bags
