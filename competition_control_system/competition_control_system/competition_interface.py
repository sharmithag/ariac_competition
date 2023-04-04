#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Order
from ariac_msgs.msg import CompetitionState
from ariac_msgs.msg import AssemblyTask, KittingTask, CombinedTask, ConveyorParts, BinParts
from std_srvs.srv import Trigger
from ariac_msgs.srv import SubmitOrder
from competition_control_system.storage_class import Orders

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
            CompetitionState, '/ariac/competition_state', 
            self.competition_state_cb, 10)

        self.starter = self.create_client(Trigger, '/ariac/start_competition')
        
        # Create subscription for conveyor_parts and bin_parts
        self.subscription = self.create_subscription(
            ConveyorParts, '/ariac/conveyor_parts', 
            self.conveyorParts_cb, 10)
        self.subscription = self.create_subscription(
            BinParts, '/ariac/bin_parts', 
            self.binParts_cb, 10)
        
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
    def conveyorParts_cb(self,msg:ConveyorParts):
        self.get_logger().info(f'Checking conveyorParts {msg.parts}')

    def binParts_cb(self,msg:BinParts):
        self.get_logger().info(f'Checking binParts {msg.bins}')
    def start_client(self):
        '''
        A method to initiate the submit order client
        '''
        self.submit_order_cli = self.create_client(SubmitOrder, '/ariac/submit_order')       
        while not self.submit_order_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SubmitOrder.Request()
        
        for key, orders in self.orders_dict.items():
            if key == '1':
                for order in orders:
                    response = self.submit_order_client_callback(order.id)
                    self.get_logger().info(f'Response for {order.id} is {response.success} and {response.message}')
            else:
                for order in orders:
                    response = self.submit_order_client_callback(order.id)
                    self.get_logger().info(f'Response for {order.id} is {response.success} and {response.message}')
                    # self.get_logger().info(f'Checking task {order.task}')
                
    def submit_order_client_callback(self, order_id):
        '''
        The callback to request the SubmitOrder service
        
        args:
            order_id: The ID of the order for which the service is requested.
            
        return: Returns the result of the service
        '''
        self.req.order_id = order_id
        self.future = self.submit_order_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        
                    
                
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
            
        self.start_client()

    def breakbeam0_cb(self, msg: BreakBeamStatus):
        if not self.object_detected and msg.object_detected:
            self.part_count += 1

        self.object_detected = msg.object_detected
