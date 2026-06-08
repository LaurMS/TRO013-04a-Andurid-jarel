"""Moodul 04a, Ülesanne 2: Odomeetria marsruut.

Ülesanne:
  Kirjuta sõlm, mis liigutab robotit täpse teekonna:
  1. Sõida 1.0 m otse ette
  2. Pöördu paremale 90°
  3. Sõida 1.0 m otse ette
  4. Pöördu paremale 90°
  5. (Robot on nüüd ~alguspunkti lähedal)

  Kasuta /odom andmeid, et teada millal iga etapp on lõpetatud.
  ÄRA kasuta time.sleep()!

Käivita:
  Terminal 1: ros2 launch yahboom_webots webots.launch.py
  Terminal 2: ros2 run marsruut marsruut
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Olekud (state machine)
OLEK_EDASI_1 = 0
OLEK_POORDE_1 = 1
OLEK_EDASI_2 = 2
OLEK_POORDE_2 = 3
OLEK_VALMIS = 4


class Marsruut(Node):

    SOIDUKIIRUS = 0.2    # m/s
    POORDEKIIRUS = 0.5   # rad/s
    VAHEMAA = 1.0        # meetrit iga sirge lõik
    POORDE_NURK = -math.pi / 2  # -90° = paremale

    def __init__(self):
        super().__init__('marsruut')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.olek = OLEK_EDASI_1
        self.algus_x = None
        self.algus_y = None
        self.algus_yaw = None
        self.praegune_x = 0.0
        self.praegune_y = 0.0
        self.praegune_yaw = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Marsruut käivitatud!')

    def odom_callback(self, msg):
        self.praegune_x = msg.pose.pose.position.x
        self.praegune_y = msg.pose.pose.position.y

        # Quaternion → yaw (pöördenurk)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.praegune_yaw = math.atan2(siny, cosy)

    def labitud_vahemaa(self):
        """Arvuta läbitud vahemaa alguspunktist."""
        if self.algus_x is None:
            return 0.0
        dx = self.praegune_x - self.algus_x
        dy = self.praegune_y - self.algus_y
        return math.sqrt(dx * dx + dy * dy)

    def poorde_nurk(self):
        """Arvuta pööratud nurk algussuunast."""
        if self.algus_yaw is None:
            return 0.0
        diff = self.praegune_yaw - self.algus_yaw
        # Normaliseeri vahemikku [-pi, pi]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def salvesta_algus(self):
        """Salvesta praegune positsioon/suund oleku alguspunktiks."""
        self.algus_x = self.praegune_x
        self.algus_y = self.praegune_y
        self.algus_yaw = self.praegune_yaw

def control_loop(self):
    cmd = Twist()

    if self.olek == OLEK_EDASI_1:

        if self.algus_x is None:
            self.salvesta_algus()

        if self.labitud_vahemaa() >= self.VAHEMAA:
            self.get_logger().info('1m läbitud, pöördun...')
            self.salvesta_algus()
            self.olek = OLEK_POORDE_1
        else:
            cmd.linear.x = self.SOIDUKIIRUS

    elif self.olek == OLEK_POORDE_1:

        if abs(self.poorde_nurk()) >= abs(self.POORDE_NURK):
            self.get_logger().info('90° pöördud, sõidan edasi...')
            self.salvesta_algus()
            self.olek = OLEK_EDASI_2
        else:
            cmd.angular.z = -self.POORDEKIIRUS

    elif self.olek == OLEK_EDASI_2:

        if self.labitud_vahemaa() >= self.VAHEMAA:
            self.get_logger().info('Teine 1m läbitud, pöördun...')
            self.salvesta_algus()
            self.olek = OLEK_POORDE_2
        else:
            cmd.linear.x = self.SOIDUKIIRUS

    elif self.olek == OLEK_POORDE_2:

        if abs(self.poorde_nurk()) >= abs(self.POORDE_NURK):
            self.get_logger().info('Marsruut lõpetatud!')
            self.olek = OLEK_VALMIS
        else:
            cmd.angular.z = -self.POORDEKIIRUS

    elif self.olek == OLEK_VALMIS:

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        self.timer.cancel()
        return

    self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Marsruut()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
