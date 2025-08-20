#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from typing import List, Tuple, Dict, Set

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# ==============================
# Settings
# ==============================
DEFAULT_PERIOD_SEC = 2.0

# Nodes we expect to see (fully-qualified names recommended)
NODE_NAMES: List[str] = [
    "/cam_sync",
    "/cam_sync_container",
    "/ouster/os_driver",
    "/rosbag2_recorder",
    "/ublox_gps_node",
    "/vectornav",
    "/vn_sensor_msgs",
    "/computer_monitor",
]

# For nodes that must publish certain topics, list them here.
# Keys must be fully-qualified node names as in NODE_NAMES.
# Topic names should be the names that appear in the ROS graph (discovery).
CAMERA_TOPICS = [
    "/cam_sync/cam0/image_raw",
    "/cam_sync/cam0/image_raw/compressed",
    "/cam_sync/cam0/meta",
    "/cam_sync/cam0/camera_info",
    "/cam_sync/cam1/image_raw",
    "/cam_sync/cam1/image_raw/compressed",
    "/cam_sync/cam1/meta",
    "/cam_sync/cam1/camera_info",
]
OUSTER_TOPICS = [
    "/ouster/os_driver/transition_event",  # Optional, not always present
    "/ouster/metadata",
    "/ouster/imu",
    "/ouster/points",
    "/ouster/telemetry",  # Optional, not always present
    "/ouster/scan",
    "/ouster/reflec_image",
    "/ouster/signal_image",
    "/ouster/nearir_image",
    "/ouster/range_image",
]
VECTORNAV_TOPICS = [
    "/vectornav/raw/ins",
    "/vectornav/raw/gps",
    "/vectornav/raw/attitude",
    "/parameter_events",
    "/vectornav/raw/gps2",
    "/vectornav/raw/time",
    "/vectornav/raw/common",
    "/vectornav/raw/imu",
]
VECTORNAV_SENSOR_TOPICS = [
    "/vectornav/temperature",
    "/vectornav/time_gps",
    "/vectornav/pose",
    "/vectornav/imu_uncompensated",
    "/vectornav/time_pps",
    "/vectornav/velocity_body",
    "/vectornav/time_startup",
    "/vectornav/imu",
    "/vectornav/pressure",
    "/vectornav/time_syncin",
    "/vectornav/magnetic",
    "/parameter_events",
    "/vectornav/gnss",
]
GPS_TOPICS = [
    "/ublox_gps_node/navpvt",
    "/ublox_gps_node/fix",
    "/ublox_gps_node/fix_velocity",
]


MUST_HAVE_TOPICS: Dict[str, List[str]] = {
    "/cam_sync": CAMERA_TOPICS,
    "/ouster/os_driver": OUSTER_TOPICS,
    "/vectornav": VECTORNAV_TOPICS,
    "/vn_sensor_msgs": VECTORNAV_SENSOR_TOPICS,
    "/ublox_gps_node": GPS_TOPICS,
}

# If True, include the set of topics each node is publishing in diagnostics.
SHOW_TOPICS: bool = True


# ==============================
# Status producer
# ==============================
class NodeMonitor:
    """
    Produces a list of DiagnosticStatus messages for monitored nodes.
    Designed to be called periodically by a ROS 2 node.
    """

    def __init__(self, node: Node, show_topics: bool = True):
        self.node = node
        self.show_topics = show_topics

    def _fq_node(self, ns: str, name: str) -> str:
        """Build a fully-qualified node name from (namespace, name)."""
        if not ns or ns == "/":
            return f"/{name}"
        if ns.endswith("/"):
            ns = ns[:-1]
        return f"{ns}/{name}"

    def _split_fqn(self, node_fqn: str) -> Tuple[str, str]:
        """
        Split a fully-qualified node name into (namespace, name).
        '/foo/bar/my_node' -> ('/foo/bar', 'my_node')
        '/my_node'         -> ('/', 'my_node')
        """
        if not node_fqn.startswith("/"):
            node_fqn = "/" + node_fqn
        parts = node_fqn.rsplit("/", 1)
        if len(parts) == 1:
            return ("/", parts[0].lstrip("/"))
        ns = parts[0] if parts[0] else "/"
        name = parts[1]
        if ns == "":
            ns = "/"
        return (ns, name)

    def _visible_nodes(self) -> Set[str]:
        seen = set()
        try:
            for name, ns in self.node.get_node_names_and_namespaces():
                seen.add(self._fq_node(ns, name))
        except Exception as e:
            self.node.get_logger().error(f"Failed to query node graph: {e}")
        return seen

    def _pubs_for_node(self, node_fqn: str) -> List[Tuple[str, List[str]]]:
        ns, name = self._split_fqn(node_fqn)
        try:
            return self.node.get_publisher_names_and_types_by_node(node_name=name, node_namespace=ns)
        except Exception as e:
            self.node.get_logger().warn(f"Could not get publishers for {node_fqn}: {e}")
            return []

    def get_statuses(self) -> List[DiagnosticStatus]:
        present_nodes = self._visible_nodes()
        statuses = []
        overall_level = DiagnosticStatus.OK
        overall_msgs = []

        for node_fqn in NODE_NAMES:
            status = DiagnosticStatus()
            status.name = f"node_monitor/{node_fqn}".replace("//", "/")
            kv: List[KeyValue] = []

            if node_fqn in present_nodes:
                pubs = self._pubs_for_node(node_fqn)
                pub_topics = sorted([t for (t, _types) in pubs])
                kv.append(KeyValue(key="visible", value="true"))
                kv.append(KeyValue(key="publisher_count", value=str(len(pub_topics))))


                missing_required = []
                if node_fqn in MUST_HAVE_TOPICS:
                    required = MUST_HAVE_TOPICS[node_fqn]
                    missing_required = sorted([t for t in required if t not in pub_topics])
                    kv.append(KeyValue(key="required_topic_count", value=str(len(required))))
                    kv.append(KeyValue(key="required_topics", value=", ".join(required)))
                    if missing_required:
                        kv.append(KeyValue(key="missing_required_topics", value=", ".join(missing_required)))

                if self.show_topics:
                    kv.append(KeyValue(key="publishers", value=", ".join(pub_topics) if pub_topics else "(none)"))

                if missing_required:
                    status.level = DiagnosticStatus.WARN
                    status.message = f"Alive, but missing required topics: {', '.join(missing_required)}"
                    overall_level = max(overall_level, DiagnosticStatus.WARN)
                    overall_msgs.append(f"{node_fqn}: missing topics")
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = "Alive"
            else:
                kv.append(KeyValue(key="visible", value="false"))
                if node_fqn in MUST_HAVE_TOPICS:
                    kv.append(KeyValue(key="required_topics", value=", ".join(MUST_HAVE_TOPICS[node_fqn])))
                status.level = DiagnosticStatus.ERROR
                status.message = "Not visible"
                overall_level = max(overall_level, DiagnosticStatus.ERROR)
                overall_msgs.append(f"{node_fqn}: not visible")

            status.values = kv
            statuses.append(status)

        summary = DiagnosticStatus()
        summary.name = "node_monitor/summary"
        summary.level = overall_level
        summary.message = "; ".join(overall_msgs) if overall_msgs else "All monitored nodes present and passing checks."
        summary.values = [
            KeyValue(key="expected_node_count", value=str(len(NODE_NAMES))),
            KeyValue(key="show_topics", value=str(self.show_topics).lower()),
        ]
        statuses.append(summary)

        return statuses


# ==============================
# ROS 2 Node wrapper & main
# ==============================
class NodeMonitorNode(Node):
    def __init__(self):
        super().__init__("node_monitor")
        self.declare_parameter("period_sec", DEFAULT_PERIOD_SEC)
        self.declare_parameter("show_topics", SHOW_TOPICS)

        show_topics = self.get_parameter("show_topics").get_parameter_value().bool_value
        self.monitor = NodeMonitor(self, show_topics=show_topics)
        self.diag_pub = self.create_publisher(DiagnosticArray, "node/diagnostics", 10)

        period = float(self.get_parameter("period_sec").get_parameter_value().double_value)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"NodeMonitorNode started. Monitoring {len(NODE_NAMES)} node(s) every "
            f"{period:.2f}s. show_topics={show_topics}"
        )

    def _tick(self):
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status = self.monitor.get_statuses()
        self.diag_pub.publish(diag)

        summary = next((s for s in diag.status if s.name == "node_monitor/summary"), None)
        if summary:
            if summary.level == DiagnosticStatus.ERROR:
                self.get_logger().warn(f"[node_monitor] ERROR: {summary.message}")
            elif summary.level == DiagnosticStatus.WARN:
                self.get_logger().warn(f"[node_monitor] WARN: {summary.message}")


def main():
    rclpy.init()
    node = NodeMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
