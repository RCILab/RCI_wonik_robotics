#!/usr/bin/env python3
"""Remove points on the robot before laser projection and obstacle detection."""
import numpy as np
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from tf_transformations import quaternion_matrix
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from wonik_navigation.ros_node import ManagedNode, spin_node


def transform_matrix(transform):
    q, p = transform.transform.rotation, transform.transform.translation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = (p.x, p.y, p.z)
    return matrix


def crop_cloud(msg, matrix, lower, upper, negative, output_frame, output_matrix=None):
    """Transform, crop and repack while preserving stamp and every non-XYZ field."""
    fields = {f.name: f for f in msg.fields}
    if any(k not in fields or fields[k].datatype not in (7, 8) or fields[k].count != 1 for k in "xyz"):
        raise ValueError("scalar float32/float64 x/y/z fields required")
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
        raise ValueError("invalid point cloud layout")
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({"names": list("xyz"),
                      "formats": [endian + ("f4" if fields[k].datatype == 7 else "f8") for k in "xyz"],
                      "offsets": [fields[k].offset for k in "xyz"], "itemsize": msg.point_step})
    # Read records as raw bytes so padding and fields such as intensity survive selection.
    records = np.ndarray((msg.height, msg.width), dtype="V%d" % msg.point_step,
                         buffer=bytes(msg.data), strides=(msg.row_step, msg.point_step)).reshape(-1)
    points = records.view(dtype)
    xyz = np.column_stack([points[k] for k in "xyz"]).astype(np.float64)
    xyz = xyz @ matrix[:3, :3].T + matrix[:3, 3]
    finite = np.isfinite(xyz).all(axis=1)
    inside = ((xyz >= lower) & (xyz <= upper)).all(axis=1)
    keep = finite & (~inside if negative else inside)
    selected = records[keep].copy()
    xyz = xyz[keep]
    if output_matrix is not None:
        xyz = xyz @ output_matrix[:3, :3].T + output_matrix[:3, 3]
    values = selected.view(dtype)
    for i, key in enumerate("xyz"):
        values[key] = xyz[:, i]
    return PointCloud2(header=Header(stamp=msg.header.stamp, frame_id=output_frame),
                       height=1, width=len(selected), fields=msg.fields,
                       is_bigendian=msg.is_bigendian, point_step=msg.point_step,
                       row_step=len(selected) * msg.point_step, data=selected.tobytes(), is_dense=True)


class SelfFilter(ManagedNode):
    def __init__(self):
        super().__init__("self_filter")
        self.input_frame = self.param("input_frame", "base_footprint")
        self.output_frame = self.param("output_frame", self.input_frame)
        self.negative = self.param("negative", True)
        self.lower = np.array([self.param("min_" + k, -1.0) for k in "xyz"])
        self.upper = np.array([self.param("max_" + k, 1.0) for k in "xyz"])
        if np.any(self.lower > self.upper):
            raise ValueError("self-filter minimum exceeds maximum")
        self.tf = tf2_ros.Buffer(node=self)
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.pub = self.create_publisher(PointCloud2, "points_self_filtered", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "points", self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg):
        try:
            stamp = Time.from_msg(msg.header.stamp)
            matrix = transform_matrix(self.tf.lookup_transform(self.input_frame, msg.header.frame_id, stamp))
            out = None
            if self.output_frame != self.input_frame:
                out = transform_matrix(self.tf.lookup_transform(self.output_frame, self.input_frame, stamp))
            self.pub.publish(crop_cloud(msg, matrix, self.lower, self.upper,
                                        self.negative, self.output_frame, out))
        except (tf2_ros.TransformException, ValueError) as exc:
            self.warn_throttle(5.0, "self_filter: dropping unusable cloud: %s", exc)


if __name__ == "__main__":
    spin_node(SelfFilter)
