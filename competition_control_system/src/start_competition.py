#!/usr/bin/env python3

import rclpy
from competition_control_system.competition_interface import CompetitionInterface

def main(args=None):
    rclpy.init(args=args)

    interface = CompetitionInterface()
    interface.start_competition()
    while rclpy.ok():
        try:
            rclpy.spin_once(interface)
            image = interface.camera_image
            interface.get_logger().info(f'Part Count: {interface.part_count}', throttle_duration_sec=2.0)

            if image is not None:
                interface.get_logger().info(interface.parse_advanced_camera_image(image), throttle_duration_sec=5.0)
        except KeyboardInterrupt:
            break
    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()