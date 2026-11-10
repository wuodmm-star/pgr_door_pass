#!/usr/bin/env python3
"""Watches /ground_truth during the demo: reports whether the footprint crossed the door, the smallest clearance to
the jambs and whether the robot stopped or turned on the spot before passing (one-go criterion)."""
import math, sys, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

W = float(sys.argv[1]) if len(sys.argv) > 1 else 0.80
T = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
XMIN, XMAX, HW, WALL = -0.25, 0.80, 0.33, 0.12


def clearance(x, y, th):
    """min distance from the jamb corners to the footprint (negative = overlap)"""
    best = 9.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            dx, dy = sx * WALL / 2 - x, sy * W / 2 - y
            u, v = math.cos(th) * dx + math.sin(th) * dy, -math.sin(th) * dx + math.cos(th) * dy
            ex, ey = max(XMIN - u, 0, u - XMAX), max(abs(v) - HW, 0)
            d = math.hypot(ex, ey) if (ex > 0 or ey > 0) else -min(u - XMIN, XMAX - u, HW - abs(v))
            best = min(best, d)
    return best


class Check(Node):
    def __init__(self):
        super().__init__('check_pass')
        self.min_clear, self.passed, self.stops, self.spins, self.t0 = 9.0, False, 0, 0, None
        self.create_subscription(Odometry, '/ground_truth', self.cb, 50)

    def cb(self, m):
        p, o, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        th = math.atan2(2 * o.w * o.z, 1 - 2 * o.z * o.z)
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.t0 = t if self.t0 is None else self.t0
        v = tw.linear.x * math.cos(th) + tw.linear.y * math.sin(th)
        if p.x + XMAX > -0.5 and p.x + XMIN < WALL / 2 + 0.2:
            self.min_clear = min(self.min_clear, clearance(p.x, p.y, th))
        if not self.passed and t - self.t0 > 10.0:
            self.stops += abs(v) < 0.02 and abs(tw.angular.z) < 0.1
            self.spins += abs(v) < 0.05 and abs(tw.angular.z) > 0.2
        if p.x + XMIN * math.cos(th) - HW * abs(math.sin(th)) > WALL / 2:
            self.passed = True
        self.t = t


rclpy.init()
n = Check()
t_wall = time.time()
while rclpy.ok() and time.time() - t_wall < T and not n.passed:
    rclpy.spin_once(n, timeout_sec=0.1)
print(f'door {W:.2f} m: passed={n.passed} min_clearance={n.min_clear * 100:.1f} cm '
      f'stop_samples={n.stops} spin_samples={n.spins}', flush=True)
