#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import statistics
import time

PERIOD_RECORD_SIZE = 5
ZERO_VEL_COMMAND = Twist()

def is_zero_velocity(msg):
    return msg.linear.x == 0.0 and msg.angular.z == 0.0

def sign(x):
    return math.copysign(1.0, x)

class RobotFeedback:
    NONE = 0
    ODOMETRY = 1
    COMMANDS = 2

class VelocitySmoother(Node):

    def __init__(self):
        super().__init__('velocity_smoother')
        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('speed_lim_v', 0.8),
                ('speed_lim_w', 5.4),
                ('accel_lim_v', 1.0),
                ('accel_lim_w', 2.0),
                ('decel_factor', 1.5),
                ('frequency', 20.0),
                ('quiet', False),
                ('robot_feedback', 0)
            ]
        )

        self.speed_lim_v = self.get_parameter('speed_lim_v').value
        self.speed_lim_w = self.get_parameter('speed_lim_w').value
        self.accel_lim_v = self.get_parameter('accel_lim_v').value
        self.accel_lim_w = self.get_parameter('accel_lim_w').value
        self.decel_factor = self.get_parameter('decel_factor').value
        self.quiet = self.get_parameter('quiet').value
        self.robot_feedback = self.get_parameter('robot_feedback').value
        self.frequency = self.get_parameter('frequency').value

        self.decel_lim_v = self.decel_factor * self.accel_lim_v
        self.decel_lim_w = self.decel_factor * self.accel_lim_w

        # Internal state
        self.input_active = False
        self.last_cb_time = self.get_clock().now()
        self.period_record = []
        self.pr_next = 0
        self.cb_avg_time = 0.1

        self.current_vel = Twist()
        self.target_vel = Twist()
        self.last_cmd_vel = Twist()

        # Publishers and Subscribers
        self.pub = self.create_publisher(Twist, 'smooth_cmd_vel', 1)
        self.create_subscription(Twist, 'cmd_vel', self.velocity_cb, 1)
        self.create_subscription(Odometry, 'robot_odom', self.odometry_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, 'robot_cmd_vel', self.robot_vel_cb, 1)

        # Timer for spin loop
        self.timer = self.create_timer(1.0 / self.frequency, self.spin)

    def velocity_cb(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_cb_time).nanoseconds * 1e-9

        if len(self.period_record) < PERIOD_RECORD_SIZE:
            self.period_record.append(dt)
        else:
            self.period_record[self.pr_next] = dt

        self.pr_next = (self.pr_next + 1) % PERIOD_RECORD_SIZE
        self.last_cb_time = now

        if len(self.period_record) <= PERIOD_RECORD_SIZE // 2:
            self.cb_avg_time = 0.1
        else:
            self.cb_avg_time = statistics.median(self.period_record)

        self.input_active = True

        # Clamp to speed limits
        self.target_vel.linear.x = max(min(msg.linear.x, self.speed_lim_v), -self.speed_lim_v)
        self.target_vel.angular.z = max(min(msg.angular.z, self.speed_lim_w), -self.speed_lim_w)

    def odometry_cb(self, msg):
        if self.robot_feedback == RobotFeedback.ODOMETRY:
            self.current_vel = msg.twist.twist

    def robot_vel_cb(self, msg):
        if self.robot_feedback == RobotFeedback.COMMANDS:
            self.current_vel = msg

    def spin(self):
        now = self.get_clock().now()
        dt = 1.0 / self.frequency

        # Check for stale input
        time_since_last_cb = (now - self.last_cb_time).nanoseconds * 1e-9
        if self.input_active and self.cb_avg_time > 0.0 and time_since_last_cb > min(3.0 * self.cb_avg_time, 0.5):
            self.input_active = False
            if not is_zero_velocity(self.target_vel):
                self.get_logger().warn(
                    f"Input became inactive, resetting target velocity: "
                    f"{self.target_vel.linear.x}, {self.target_vel.angular.z}"
                )
                self.target_vel = ZERO_VEL_COMMAND

        # Check feedback deviation
        period_buffer = 2.0
        v_dev_low = self.last_cmd_vel.linear.x - self.decel_lim_v * dt * period_buffer
        v_dev_high = self.last_cmd_vel.linear.x + self.accel_lim_v * dt * period_buffer
        w_dev_low = self.last_cmd_vel.angular.z - self.decel_lim_w * dt * period_buffer
        w_dev_high = self.last_cmd_vel.angular.z + self.accel_lim_w * dt * period_buffer

        v_diff = not (v_dev_low <= self.current_vel.linear.x <= v_dev_high)
        w_diff = not (w_dev_low <= self.current_vel.angular.z <= w_dev_high)

        if self.robot_feedback != RobotFeedback.NONE and self.input_active and self.cb_avg_time > 0.0:
            if time_since_last_cb > 5.0 * self.cb_avg_time or v_diff or w_diff:
                if not self.quiet:
                    self.get_logger().warn(
                        f"Switching to robot feedback due to deviation: "
                        f"{time_since_last_cb:.2f}s, "
                        f"Δv={self.current_vel.linear.x - self.last_cmd_vel.linear.x:.3f}, "
                        f"Δw={self.current_vel.angular.z - self.last_cmd_vel.angular.z:.3f}"
                    )
                self.last_cmd_vel = self.current_vel

        cmd_vel = Twist()
        if (self.target_vel.linear.x != self.last_cmd_vel.linear.x or
            self.target_vel.angular.z != self.last_cmd_vel.angular.z):
            # Compute increments
            v_inc = self.target_vel.linear.x - self.last_cmd_vel.linear.x
            w_inc = self.target_vel.angular.z - self.last_cmd_vel.angular.z

            if self.robot_feedback == RobotFeedback.ODOMETRY and \
               self.current_vel.linear.x * self.target_vel.linear.x < 0.0:
                max_v_inc = self.decel_lim_v * dt
            else:
                max_v_inc = (self.accel_lim_v if v_inc * self.target_vel.linear.x > 0 else self.decel_lim_v) * dt

            if self.robot_feedback == RobotFeedback.ODOMETRY and \
               self.current_vel.angular.z * self.target_vel.angular.z < 0.0:
                max_w_inc = self.decel_lim_w * dt
            else:
                max_w_inc = (self.accel_lim_w if w_inc * self.target_vel.angular.z > 0 else self.decel_lim_w) * dt

            # Normalize and apply geometric constraint
            ma = math.hypot(v_inc, w_inc)
            mb = math.hypot(max_v_inc, max_w_inc)

            av = abs(v_inc) / ma if ma else 0
            aw = abs(w_inc) / ma if ma else 0
            bv = max_v_inc / mb if mb else 0
            bw = max_w_inc / mb if mb else 0

            theta = math.atan2(bw, bv) - math.atan2(aw, av)

            if theta < 0:
                max_v_inc = (max_w_inc * abs(v_inc)) / abs(w_inc) if w_inc else max_v_inc
            else:
                max_w_inc = (max_v_inc * abs(w_inc)) / abs(v_inc) if v_inc else max_w_inc

            # Limit increments
            cmd_vel.linear.x = self.last_cmd_vel.linear.x + sign(v_inc) * min(abs(v_inc), max_v_inc)
            cmd_vel.angular.z = self.last_cmd_vel.angular.z + sign(w_inc) * min(abs(w_inc), max_w_inc)

            self.last_cmd_vel = cmd_vel
        elif self.input_active:
            # Resend last command
            cmd_vel = self.last_cmd_vel

        self.pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()