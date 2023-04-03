#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Order
from ariac_msgs.msg import CompetitionState
from std_srvs.srv import Trigger
from ariac_msgs.srv import SubmitOrder
from competition_tutorials.storage_class import Orders

from rclpy import qos
from ariac_msgs.msg import BreakBeamStatus


class CompetitionInterface(Node):
    # Dictionary to convert competition_state constants to strings
    states = {
        CompetitionState.IDLE: 'idle',
        CompetitionState.READY: 'ready',
        CompetitionState.STARTED: 'started',
        CompetitionState.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionState.ENDED: 'ended',
    }

    def __init__(self):
        super().__init__('start_competition_node')

        self.competition_state = None
        self.orders_dict = {"0": [], "1": []}
        self.subscription = self.create_subscription(
            CompetitionState,
            '/ariac/competition_state',
            self.competition_state_cb,
            10)

        self.starter = self.create_client(Trigger, '/ariac/start_competition')
        self.submit_order_cli = self.create_client(SubmitOrder, '/ariac/submit_order')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SubmitOrder.Request()
        # Create subscription for breakbeam_0
        self.part_count = 0
        self.object_detected = False
        self.break_beam_sub = self.create_subscription(
            BreakBeamStatus,
            '/ariac/sensors/breakbeam_0/status',
            self.breakbeam0_cb,
            qos.qos_profile_sensor_data)
        # Create subscription for retreiving orders
        self.retrieve_order_sub = self.create_subscription(
            Order,
            '/ariac/orders',
            self.retrieve_order_cb,
            10)
        # service to submit orders

        # self.submit_orders_srv = self.create_service(
        #     SubmitOrder, '/ariac/submit_order', self.submit_order_cb)
    def submit_order_client_callback(self):
        for i in self.orders_dict["1"]:
            self.req.order_id = i.id
        for j in self.orders_dict["0"]:
            self.req.order_id = j.id
    def competition_state_cb(self, msg: CompetitionState):
        # Log if competition state has changed
        if self.competition_state != msg.competition_state:
            self.get_logger().info(
                f'Competition state is: {self.states[msg.competition_state]}',
                throttle_duration_sec=1.0)
        self.competition_state = msg.competition_state

    def retrieve_order_cb(self, msg: Order):
        if msg.type == 0:
            order = Orders(msg.id, msg.priority, msg.type, msg.kitting_task)

        if msg.type == 1:
            order = Orders(msg.id, msg.priority, msg.type, msg.assembly_task)
        if msg.type == 2:
            order = Orders(msg.id, msg.priority, msg.type, msg.combined_task)
        if msg.priority == 1:
            self.orders_dict["1"].append(order)
        if msg.priority == 0:
            self.orders_dict["0"].append(order)

        self.get_logger().info('I heard: "%s"' % msg.id)

    # Service callback for submitting orders
    # def submit_order_cb(self, request, response):
    #     for i, j in self.orders_dict.items():
    #         if i == 1:
    #             for j in self.orders_dict[i]:
    #                 response.success = 1
    #                 response.message = j.id + " Task Completed "
    #                 self.orders_dict[i].pop[j]
    #                 self.get_logger().info('Requested  \n%d' % (request.a, request.b))
    #         else:
    #             for k in self.orders_dict[i]:
    #                 response.success = 1
    #                 response.message = k.id + " Task Completed "
    #                 self.orders_dict[i].pop[k]
    #     return response

    def start_competition(self):
        self.get_logger().info('Waiting for competition to be ready')

        # Wait for competition to be ready
        while (self.competition_state != CompetitionState.READY):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return

        self.get_logger().info('Competition is ready. Starting...')

        # Call ROS service to start competition
        while not self.starter.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ariac/start_competition to be available...')

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self.starter.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().info('Unable to start competition')
        while (self.competition_state != CompetitionState.ORDER_ANNOUNCEMENTS_DONE):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return

    def breakbeam0_cb(self, msg: BreakBeamStatus):
        if not self.object_detected and msg.object_detected:
            self.part_count += 1

        self.object_detected = msg.object_detected
