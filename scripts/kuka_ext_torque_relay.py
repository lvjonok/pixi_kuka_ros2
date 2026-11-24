#!/usr/bin/env python3
"""Republish KUKA's external-torque estimate from lbr_state as a sensor_msgs/JointState.

lbr_fri_idl is built only in this workspace; a reader in another environment takes
kuka_external_torque: effort is LBRState.external_torque in Nm, names lbr_A1 to lbr_A7.
"""

import rclpy
from lbr_fri_idl.msg import LBRState
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

NAMES = [f"lbr_A{i}" for i in range(1, 8)]


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("kuka_ext_torque_relay")
    pub = node.create_publisher(JointState, "kuka_external_torque", qos_profile_sensor_data)

    def relay(msg: LBRState) -> None:
        out = JointState(name=NAMES, effort=list(msg.external_torque))
        out.header.stamp = node.get_clock().now().to_msg()
        pub.publish(out)

    node.create_subscription(LBRState, "lbr_state", relay, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
