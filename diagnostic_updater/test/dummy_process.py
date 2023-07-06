#!/usr/bin/env python3

from diagnostic_updater import Updater
import rclpy


def main():
    rclpy.init()

    node = rclpy.create_node('talker')
    updater = Updater(node)
    updater.add('do_nothing', lambda stat: stat)
    updater.verbose = True
    node.create_timer(0.1, lambda: updater.update())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
