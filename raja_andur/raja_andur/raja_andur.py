#!/usr/bin/env python3
"""
Moodul 04a, Ülesanne 1: Raja andur — lidar sektorite analüüs.

Ülesanne:
  Kirjuta sõlm, mis jagab lidari 360° vaate 5 sektoriks ja
  trükib iga sekundi tagant tabeli kaugustest.

Nõuded:
  - Subscribi /scan teemale
  - Jaga vaade 5 sektoriks: vasak sein, ette-vasak,
    otse ette, ette-parem, parem sein
  - Trüki tabel iga sekundi tagant
  - Märgista:
      [LÄHEDAL] < 0.5m
      [HOIATUS] < 1.0m
      [OK] >= 1.0m
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RajaAndur(Node):

    def __init__(self):
        super().__init__('raja_andur')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.viimane_scan = None

        # Trüki tabel 1 kord sekundis
        self.timer = self.create_timer(
            1.0,
            self.print_table
        )

        self.get_logger().info('Raja andur käivitatud!')

    def scan_callback(self, msg):
        self.viimane_scan = msg

    def sektori_min(self, ranges, algus, lopp,
                    range_min, range_max):
        """Leia minimaalne kehtiv kaugus sektoris."""

        kehtivad = [
            ranges[i % len(ranges)]
            for i in range(algus, lopp)
            if (
                range_min <= ranges[i % len(ranges)] <= range_max
                and not math.isinf(ranges[i % len(ranges)])
                and not math.isnan(ranges[i % len(ranges)])
            )
        ]

        return min(kehtivad) if kehtivad else float('inf')

    def margista(self, kaugus):
        """Tagasta märgis kauguse põhjal."""

        if kaugus < 0.5:
            return '[LÄHEDAL]'
        elif kaugus < 1.0:
            return '[HOIATUS]'
        else:
            return '[OK]'

    def print_table(self):

        if self.viimane_scan is None:
            return

        msg = self.viimane_scan
        ranges = msg.ranges

        # 720 kiirega lidari sektorid

        # ~90° vasak ±20°
        vasak_sein = self.sektori_min(
            ranges, 520, 560,
            msg.range_min, msg.range_max
        )

        # ~20–45° vasak
        ette_vasak = self.sektori_min(
            ranges, 400, 450,
            msg.range_min, msg.range_max
        )

        # ±10° otse ette
        otse_ette = self.sektori_min(
            ranges, 340, 380,
            msg.range_min, msg.range_max
        )

        # ~20–45° parem
        ette_parem = self.sektori_min(
            ranges, 270, 320,
            msg.range_min, msg.range_max
        )

        # ~90° parem ±20°
        parem_sein = self.sektori_min(
            ranges, 160, 200,
            msg.range_min, msg.range_max
        )

        self.get_logger().info(
            f'\n=== Raja andurid ===\n'
            f'Vasak sein:    {vasak_sein:.2f} m  {self.margista(vasak_sein)}\n'
            f'Ette-vasak:    {ette_vasak:.2f} m  {self.margista(ette_vasak)}\n'
            f'Otse ette:     {otse_ette:.2f} m  {self.margista(otse_ette)}\n'
            f'Ette-parem:    {ette_parem:.2f} m  {self.margista(ette_parem)}\n'
            f'Parem sein:    {parem_sein:.2f} m  {self.margista(parem_sein)}\n'
            f'==================='
        )


def main(args=None):
    rclpy.init(args=args)

    node = RajaAndur()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
