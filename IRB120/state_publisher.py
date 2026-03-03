from math import sin, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

class StatePublisher(Node):
    """
    Publishes:
      - /joint_states with names that match the IRB120 URDF.
    """
    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        self.get_logger().info(f"{self.get_name()} started (30 Hz)")

        # Angles (rad) & motion parameters
        self.t = 0.0               # time/phase for nice sinusoid motions

        # URDF limits
        self.j1_min, self.j1_max = -2.87979, 2.87979
        self.j2_min, self.j2_max = -1.91986, 1.91986
        self.j3_min, self.j3_max = -1.91986, 1.22173
        self.j4_min, self.j4_max = -2.79253, 2.79253
        self.j5_min, self.j5_max = -2.094395, 2.094395
        self.j6_min, self.j6_max = -6.98132, 6.98132
        self.g_min, self.g_max = 0.0, 0.04

        # 30 Hz timer
        self.timer = self.create_timer(1.0/30.0, self._on_timer)

    def _on_timer(self):
        now = self.get_clock().now().to_msg()

        # Generate smooth positions within limits using sin()
        def mid_span(lo, hi):
            return (lo + hi) * 0.5, (hi - lo) * 0.5

        m2, a2 = mid_span(self.j2_min, self.j2_max)
        m3, a3 = mid_span(self.j3_min, self.j3_max)
        m4, a4 = mid_span(self.j4_min, self.j4_max)
        m5, a5 = mid_span(self.j5_min, self.j5_max)

        m6, a6 = mid_span(self.j6_min, self.j6_max)
        mg, ag = mid_span(self.g_min, self.g_max)

        # Sweep speeds (rad/s equivalent via phase increment)
        self.t += 0.02

        joint1 = (self.t * 0.7) % (2*pi)     # free rotate
        # wrap to [-pi, pi] for aesthetics (optional)
        if joint1 > pi:
            joint1 -= 2*pi

        joint2 = m2 + a2 * sin(self.t * 0.9)
        joint3 = m3 + a3 * sin(self.t * 0.6 + 1.1)
        joint4 = m4 + a4 * sin(self.t * 0.8 + 2.0)
        joint5 = m5 + a5 * sin(self.t * 1.1 + 0.5)
        joint6 = m6 + a6 * sin(self.t * 1.3 + 0.8)
        gripper1 = mg + ag * sin(self.t * 1.6)
        gripper2 = mg + ag * sin(self.t * 1.6)

        # -------- publish /joint_states --------
        jointstate = JointState()
        jointstate.header.stamp = now
        jointstate.name = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'gripper_finger_joint1',
            'gripper_finger_joint2',
        ]
        jointstate.position = [joint1, joint2, joint3, joint4, joint5, joint6, gripper1, gripper2]
        self.joint_pub.publish(jointstate)

def main():
    rclpy.init()
    node = StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
