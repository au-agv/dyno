 #############################################################################
 #                            _     _     _     _                            #
 #                           / \   / \   / \   / \                           #
 #                          ( D ) ( Y ) ( N ) ( O )                          #
 #                           \_/   \_/   \_/   \_/                           #
 #                                                                           #
 #              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
 #############################################################################

import rclpy
import sys
import select
import termios
import threading
import tty

from roto_interfaces.msg import ControlEffortStamped
from rclpy.node import Node

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Chrono!
---------------------------
"""


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        self.timer = self.create_timer(1.0 / 1000.0,
                                       self.timer_callback)
        self.throttle_publisher = self.create_publisher(ControlEffortStamped,
                                                        'throttle',
                                                        10)
        self.brake_publisher = self.create_publisher(ControlEffortStamped,
                                                     'brake',
                                                     10)
        self.steering_publisher = self.create_publisher(ControlEffortStamped,
                                                        'steering',
                                                        10)

        self.throttle_message = ControlEffortStamped()
        self.brake_message = ControlEffortStamped()
        self.steering_message = ControlEffortStamped()

        self.key_thread = threading.Thread(target=self._get_key_thread)
        self.key_thread.start()

        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0

    def _get_key_thread(self):

        while (True):
            key = self._get_key()
            if key in ['w', 'a', 's', 'd']:
                match key:
                    case 'w':
                        self.throttle += 0.01 if self.throttle <= 1.0 else 0.0
                        if self.throttle <= 0.005:
                            self.brake -= 0.01
                        else:
                            self.brake = 0.0
                    case 's':
                        self.throttle -= 0.01 if self.throttle >= 0.0 else 0.0
                        if self.throttle <= 0.005:
                            self.brake += 0.01
                        else:
                            self.brake = 0.0
                    case 'a':
                        self.steer += 0.01 if self.steer <= 1.0 else 0.0
                    case 'd':
                        self.steer -= 0.01 if self.steer >= -1.0 else 0.0

                print(f'{self.throttle}, {self.brake}, {self.steer}')

                self.throttle_message.effort = self.throttle
                self.brake_message.effort = self.brake
                self.steering_message.effort = self.steer

            if (key == '\x03'):
                break

    def _get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def timer_callback(self):

        self.throttle_publisher.publish(self.throttle_message)
        self.brake_publisher.publish(self.brake_message)
        self.steering_publisher.publish(self.steering_message)


def get_velocity_str(speed,
                     turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)
