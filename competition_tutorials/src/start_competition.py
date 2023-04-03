#!/usr/bin/env python3

import rclpy
from competition_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    rclpy.init(args=args)

    interface = CompetitionInterface()
    interface.start_competition()
    interface.submit_order_client_callback()
    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()