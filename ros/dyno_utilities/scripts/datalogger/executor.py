#!/usr/bin/env python3

import rclpy

from dyno_utilities.datalogging import DataloggerNode


def main():

    rclpy.init()

    node = DataloggerNode('datalogger')

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
