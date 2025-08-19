#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from typing import List, Tuple, Dict, Set

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# ==============================
# Configuration (hard-coded)
# ==============================

# Nodes we expect to see (fully-qualified names recommended)
NODE_NAMES: List[str] = [
    "/cam_sync",
    "/cam_sync_container",
    "/ouster/os_driver",
    "/rosbag2_recorder",
    "/ublox_gps_node",
    "/vectornav",
    "/vn_sensor_msgs",
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
    "/vectornav/raw/common",
    "/vectornav/raw/time",
    "/vectornav/time_startup",
    "/vectornav/time_gps",
    "/vectornav/raw/imu",
    "/vectornav/time_syncin",
    "/vectornav/raw/gps",
    "/vectornav/time_pps",
    "/vectornav/imu",
    "/vectornav/gnss",
    "/vectornav/raw/attitude",
    "/vectornav/imu_uncompensated",
    "/vectornav/raw/ins",
    "/vectornav/magnetic",
    "/vectornav/raw/gps2",
    "/vectornav/velocity_aiding",
    "/vectornav/temperature",
    "/vectornav/pressure",
    "/vectornav/velocity_body",
    "/vectornav/pose",
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
    "/ublox_gps_node": GPS_TOPICS,
}

# If True, include the set of topics each node is publishing in diagnostics.
SHOW_TOPICS: bool = True

# How often to check (seconds)
CHECK_PERIOD_SEC: float = 2.0


# ==============================
# Helper utilities
# ==============================


def fq_node(ns: str, name: str) -> str:
    """Build a fully-qualified node name from (namespace, name)."""
    if not ns or ns == "/":
        return f"/{name}"
    if ns.endswith("/"):
        ns = ns[:-1]
    return f"{ns}/{name}"


def split_fqn(node_fqn: str) -> Tuple[str, str]:
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


# ==============================
# Node monitor
# ==============================


class NodeMonitorDiag(Node):
    def __init__(self):
        super().__init__("node_monitor_diag")

        # Publisher for diagnostic array (standard tooling listens on this)
        # Use relative name so it respects remapping; defaults to '/diagnostics'
        self.diag_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)

        # Timer for periodic checks
        self.timer = self.create_timer(CHECK_PERIOD_SEC, self.check_once)

        self.get_logger().info(
            f"NodeMonitorDiag started. Monitoring {len(NODE_NAMES)} node(s) every "
            f"{CHECK_PERIOD_SEC:.2f}s. SHOW_TOPICS={SHOW_TOPICS}"
        )

    # --- Graph queries (discovery-only, zero user-topic bandwidth) ---
    def visible_nodes(self) -> Set[str]:
        seen = set()
        try:
            for name, ns in self.get_node_names_and_namespaces():
                seen.add(fq_node(ns, name))
        except Exception as e:
            self.get_logger().error(f"Failed to query node graph: {e}")
        return seen

    def pubs_for_node(self, node_fqn: str) -> List[Tuple[str, List[str]]]:
        ns, name = split_fqn(node_fqn)
        try:
            # Discovery information only; does not subscribe or consume user data.
            return self.get_publisher_names_and_types_by_node(name=name, namespace=ns)
        except Exception as e:
            self.get_logger().warn(f"Could not get publishers for {node_fqn}: {e}")
            return []

    # --- Core check ---
    def check_once(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        present_nodes = self.visible_nodes()

        overall_level = DiagnosticStatus.OK
        overall_msgs = []

        for node_fqn in NODE_NAMES:
            status = DiagnosticStatus()
            status.name = f"node_monitor/{node_fqn}"
            status.hardware_id = ""  # optional
            kv: List[KeyValue] = []

            if node_fqn in present_nodes:
                # Node is visible
                pubs = self.pubs_for_node(node_fqn)
                pub_topics = sorted([t for (t, _types) in pubs])
                kv.append(KeyValue(key="visible", value="true"))
                kv.append(KeyValue(key="publisher_count", value=str(len(pub_topics))))

                missing_required = []
                if node_fqn in MUST_HAVE_TOPICS:
                    required = MUST_HAVE_TOPICS[node_fqn]
                    missing_required = sorted(
                        [t for t in required if t not in pub_topics]
                    )
                    kv.append(
                        KeyValue(key="required_topic_count", value=str(len(required)))
                    )
                    kv.append(
                        KeyValue(key="required_topics", value=", ".join(required))
                    )
                    if missing_required:
                        kv.append(
                            KeyValue(
                                key="missing_required_topics",
                                value=", ".join(missing_required),
                            )
                        )

                # Include all publisher topics (optional)
                if SHOW_TOPICS:
                    kv.append(
                        KeyValue(
                            key="publishers",
                            value=", ".join(pub_topics) if pub_topics else "(none)",
                        )
                    )

                # Level/message
                if missing_required:
                    status.level = DiagnosticStatus.WARN
                    status.message = f"Alive, but missing required topics: {', '.join(missing_required)}"
                    overall_level = max(overall_level, DiagnosticStatus.WARN)
                    overall_msgs.append(f"{node_fqn}: missing topics")
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = "Alive"
                    overall_level = max(overall_level, DiagnosticStatus.OK)

            else:
                # Node is missing
                kv.append(KeyValue(key="visible", value="false"))
                if node_fqn in MUST_HAVE_TOPICS:
                    kv.append(
                        KeyValue(
                            key="required_topics",
                            value=", ".join(MUST_HAVE_TOPICS[node_fqn]),
                        )
                    )

                status.level = DiagnosticStatus.ERROR
                status.message = "Not visible"
                overall_level = max(overall_level, DiagnosticStatus.ERROR)
                overall_msgs.append(f"{node_fqn}: not visible")

            status.values = kv
            diag_array.status.append(status)

        # Add an overall summary status entry for dashboards
        overall = DiagnosticStatus()
        overall.name = "node_monitor/summary"
        overall.level = overall_level
        if overall_msgs:
            overall.message = "; ".join(overall_msgs)
        else:
            overall.message = "All monitored nodes present and passing checks."
        overall.values = [
            KeyValue(key="expected_node_count", value=str(len(NODE_NAMES))),
            KeyValue(key="period_sec", value=f"{CHECK_PERIOD_SEC:.2f}"),
            KeyValue(key="show_topics", value=str(SHOW_TOPICS).lower()),
        ]
        diag_array.status.append(overall)

        # Publish once per timer tick
        self.diag_pub.publish(diag_array)

        # Log succinctly when there is a problem
        if overall_level == DiagnosticStatus.ERROR:
            self.get_logger().warn(f"[node_monitor] ERROR: {overall.message}")
        elif overall_level == DiagnosticStatus.WARN:
            self.get_logger().warn(f"[node_monitor] WARN: {overall.message}")


def main():
    rclpy.init()
    node = NodeMonitorDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
