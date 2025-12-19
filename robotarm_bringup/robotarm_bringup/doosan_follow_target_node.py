import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import DR_init

class DoosanFollowTarget(Node):
    def __init__(self):
        super().__init__("doosan_follow_target")

        # ---- params ----
        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "e0509")
        self.declare_parameter("topic", "/hand_target_point")
        self.declare_parameter("vel", 10.0)
        self.declare_parameter("acc", 20.0)
        self.declare_parameter("dt", 0.3)
        self.declare_parameter("deadband_mm", 3.0)

        robot_id = self.get_parameter("robot_id").value
        robot_model = self.get_parameter("robot_model").value
        topic = self.get_parameter("topic").value

        # ✅ 핵심: DR_init에 Node 바인딩
        DR_init.__dsr__id = robot_id
        DR_init.__dsr__model = robot_model
        DR_init.__dsr__node = self

        # 👉 이 다음에 import 해야 g_node가 살아있음
        from DSR_ROBOT2 import movel, get_current_posx
        self.movel = movel
        self.get_current_posx = get_current_posx

        self.vel = float(self.get_parameter("vel").value)
        self.acc = float(self.get_parameter("acc").value)
        self.deadband = float(self.get_parameter("deadband_mm").value)

        self.sub = self.create_subscription(
            PointStamped,
            topic,
            self.cb,
            10
        )

        self.prev = None
        self.get_logger().info("[OK] Doosan follow target ready")

    def cb(self, msg: PointStamped):
        x = msg.point.x * 1000.0
        y = msg.point.y * 1000.0
        z = msg.point.z * 1000.0

        if self.prev is not None:
            dx = abs(x - self.prev[0])
            dy = abs(y - self.prev[1])
            dz = abs(z - self.prev[2])
            if max(dx, dy, dz) < self.deadband:
                return

        self.prev = (x, y, z)

        try:
            self.movel([x, y, z, 0, 180, 0], vel=self.vel, acc=self.acc)
            self.get_logger().info(f"[MOVE] {x:.1f},{y:.1f},{z:.1f}")
        except Exception as e:
            self.get_logger().error(str(e))


def main():
    rclpy.init()
    node = DoosanFollowTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
