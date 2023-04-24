#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Order
from ariac_msgs.msg import CompetitionState
from ariac_msgs.msg import AssemblyTask, KittingTask, CombinedTask, ConveyorParts, BinParts
from ariac_msgs.msg import (Part as PartMsg, PartPose as PartPoseMsg, AssemblyPart as AssemblyPartMsg,
                            AGVStatus as AGVStatusMsg, AssemblyTask as AssemblyTaskMsg)
from std_srvs.srv import Trigger
from ariac_msgs.srv import SubmitOrder
from competition_control_system.storage_class import Orders, Parts
from rclpy import qos
from ariac_msgs.msg import BreakBeamStatus
from ariac_msgs.msg import AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg

from competition_control_system.utils import (
    multiply_pose,
    rpy_from_quaternion,
    rad_to_deg_str,
    AdvancedLogicalCameraImage
)
class CompetitionInterface(Node):
    # Dictionary to convert competition_state constants to strings
    states = {
        CompetitionState.IDLE: 'idle',
        CompetitionState.READY: 'ready',
        CompetitionState.STARTED: 'started',
        CompetitionState.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionState.ENDED: 'ended',
    }
    '''Dictionary for converting Part color constants to strings'''
    _part_colors = {
        PartMsg.RED: 'red',
        PartMsg.BLUE: 'blue',
        PartMsg.GREEN: 'green',
        PartMsg.ORANGE: 'orange',
        PartMsg.PURPLE: 'purple',
    }
    '''Dictionary for converting Part color constants to emojis'''

    _part_colors_emoji = {
        PartMsg.RED: '🟥',
        PartMsg.BLUE: '🟦',
        PartMsg.GREEN: '🟩',
        PartMsg.ORANGE: '🟧',
        PartMsg.PURPLE: '🟪',
    }
    '''Dictionary for converting Part type constants to strings'''

    _part_types = {
        PartMsg.BATTERY: 'battery',
        PartMsg.PUMP: 'pump',
        PartMsg.REGULATOR: 'regulator',
        PartMsg.SENSOR: 'sensor',
        
    }
    '''Dictionary for converting AGVDestination constants to strings'''

    _destinations = {
        AGVStatusMsg.KITTING: 'kitting station',
        AGVStatusMsg.ASSEMBLY_FRONT: 'front assembly station',
        AGVStatusMsg.ASSEMBLY_BACK: 'back assembly station',
        AGVStatusMsg.WAREHOUSE: 'warehouse',
    }
    '''Dictionary for converting AssemblyTask constants to strings'''
    _stations = {
        AssemblyTaskMsg.AS1: 'assembly station 1',
        AssemblyTaskMsg.AS2: 'assembly station 2',
        AssemblyTaskMsg.AS3: 'assembly station 3',
        AssemblyTaskMsg.AS4: 'assembly station 4',
    }
    def __init__(self):
        super().__init__('start_competition_node')
        self.partObj = Parts()
        # self.bin_num_list = [1,2,3,4,5,6,7,8]
        self.competition_state = None
        self.orders_dict = {"0": [], "1": []}
        self.final_bin_parts = {"1": [], "2": [],"3": [], "4": [],"5": [], "6": [],"7": [], "8": []}
        self.subscription = self.create_subscription(
            CompetitionState, '/ariac/competition_state', 
            self.competition_state_cb, 10)

        self.starter = self.create_client(Trigger, '/ariac/start_competition')
        
         # Create subscription for conveyor_parts 

        self.subscription = self.create_subscription(
            ConveyorParts, '/ariac/conveyor_parts', 
            self.conveyorParts_cb, 1)
        # Created a subscription for bin parts
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
        
        # Subscriber to the logical camera topic
        self._advanced_camera0_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            '/ariac/sensors/advanced_camera_0/image',
            self._advanced_camera0_cb,
            qos.qos_profile_sensor_data)

        # Store each camera image as an AdvancedLogicalCameraImage object
        self._camera_image: AdvancedLogicalCameraImage = None
        
        
        # Create subscription for retreiving orders
        self.retrieve_order_sub = self.create_subscription(
            Order,
            '/ariac/orders',
            self.retrieve_order_cb,
            10)
        
    @property
    def camera_image(self):
        return self._camera_image

    @property
    def conveyor_part_count(self):
        return self._conveyor_part_count
       
    def conveyorParts_cb(self,msg:ConveyorParts):
        
        # self.get_logger().info(f'Checking conveyorParts {msg.parts}')
        # self.get_logger().info(f'Printing conveyorParts {msg.parts[0].part.color }')
        for i in range (len(msg.parts)):
            self.partObj.partsDict["conveyor"].append([msg.parts[i].part.color, msg.parts[i].part.type, msg.parts[i].quantity])
        self.partObj.partsDict["conveyor"]=list(set(tuple(row) for row in self.partObj.partsDict["conveyor"]))
        # self.get_logger().info(f'Checking conveyorParts {self.partObj.print_dict()}')

    def binParts_cb(self,msg:BinParts):
        # self.get_logger().info(f'Checking binParts {msg.bins}')
        for i in range (len(msg.bins)):
            self.partObj.partsDict["bin"].append([msg.bins[i].parts[0].part.color, msg.bins[i].parts[0].part.type, msg.bins[i].parts[0].quantity,msg.bins[i].bin_number])
            self.partObj.partsDict["bin"]=list(set(tuple(row) for row in self.partObj.partsDict["bin"]))
        # self.get_logger().info(f'Checking binParts {self.partObj.print_dict()}')
        
    def collecting_parts(self):
        '''collecting_parts_cb method : collect parts from conveyor belt and put it on the bins
        '''
        self.get_logger().info(f'Checking Collected Parts {self.partObj.print_dict()}')

        for i in range (len(self.partObj.partsDict["bin"])):
            #accessing bin_number and appending to final bin parts
            if self.final_bin_parts[f'{self.partObj.partsDict["bin"][i][3]}'] == []:
                self.final_bin_parts[f'{self.partObj.partsDict["bin"][i][3]}'].append([self.partObj.partsDict["bin"][i][0],self.partObj.partsDict["bin"][i][1],self.partObj.partsDict["bin"][i][2]])
            # self.get_logger().info(f'Bin number removed : {self.partObj.partsDict["bin"][i][3]}')
            # self.bin_num_list.remove(self.partObj.partsDict["bin"][i][3])
            
            
        #PERFORM PICK AND PLACE (pending) and update final_bin_parts dict
        for j in range (len(self.partObj.partsDict["conveyor"])):
            for k in range (1, len(self.final_bin_parts)+1):
                
                if self.final_bin_parts[f'{k}'] == []:
                    self.get_logger().info(f'CHECKING KEY VALUE : {k}')
                    self.final_bin_parts[f'{k}'].append(list(self.partObj.partsDict["conveyor"][j]))
                    break
        # self.get_logger().info(f'Checking end bin parts {self.final_bin_parts}')  
    
    
    def comparing_orders_parts(self):
        # self.get_logger().info(f'Checking end bin parts before {self.final_bin_parts}')  

        for i in self.orders_dict["0"]:
            for j in range(len(i.task.parts)):
                # self.get_logger().info(f'Tasks in orders with {i.task.parts[j].part}')
                
                for k,v in self.final_bin_parts.items():
                    if v != [] and v[0][2] > 0 :       
                                      
                        # self.get_logger().info(f'checking sublist of list {v[0]} and {[i.task.parts[j].part.color,i.task.parts[j].part.type]}')
                        if(all(x in v[0] for x in [i.task.parts[j].part.color,i.task.parts[j].part.type])):
                            # self.get_logger().info("True")
                            v[0][2] = v[0][2] - 1
                    elif v != []:
                        # Create a client for submitting order
                        self.submit_order_cli = self.create_client(SubmitOrder, '/ariac/submit_order')       
                        while not self.submit_order_cli.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('service not available, waiting again...')
                        self.req = SubmitOrder.Request()
                        response = self.submit_order_client_callback(i.id)
                        response.success = False
        # self.get_logger().info(f'Checking end bin after parts {self.final_bin_parts}')  
                    
                    
                    
            # self.get_logger().info(f'Tasks in orders with part {i.task.parts.part}')
                # self.get_logger().info(f'Tasks in orders with parts[0] {i.task.parts[0]}')
        
        # for i in self.orders_dict["0"]:
        #     for j in range(len(i.task.parts)):
        #         self.get_logger().info(f'Tasks in orders with {i.task.parts[j].part}')
        
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
        # self.get_logger().info(f'Received order for {msg} ')

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
            
       

    def parse_advanced_camera_image(self, image: AdvancedLogicalCameraImage) -> str:
        '''
        Parse an AdvancedLogicalCameraImage message and return a string representation.
        '''

        if len(image._part_poses) == 0:
            return 'No parts detected'

        output = '\n\n'
        for i, part_pose in enumerate(image._part_poses):
            part_pose: PartPoseMsg
            output += '==========================\n'
            part_color = CompetitionInterface._part_colors[part_pose.part.color].capitalize()
            part_color_emoji = CompetitionInterface._part_colors_emoji[part_pose.part.color]
            part_type = CompetitionInterface._part_types[part_pose.part.type].capitalize()
            output += f'Part {i+1}: {part_color_emoji} {part_color} {part_type}\n'
            output += '--------------------------\n'
            output += 'Camera Frame\n'
            output += '--------------------------\n'

            output += '  Position:\n'
            output += f'    x: {part_pose.pose.position.x:.3f} (m)\n'
            output += f'    y: {part_pose.pose.position.y:.3f} (m)\n'
            output += f'    z: {part_pose.pose.position.z:.3f} (m)\n'

            roll, pitch, yaw = rpy_from_quaternion(part_pose.pose.orientation)
            output += '  Orientation:\n'
            output += f'    roll: {rad_to_deg_str(roll)}\n'
            output += f'    pitch: {rad_to_deg_str(pitch)}\n'
            output += f'    yaw: {rad_to_deg_str(yaw)}\n'

            part_world_pose = multiply_pose(image._sensor_pose, part_pose.pose)
            output += '--------------------------\n'
            output += 'World Frame\n'
            output += '--------------------------\n'

            output += '  Position:\n'
            output += f'    x: {part_world_pose.position.x:.3f} (m)\n'
            output += f'    y: {part_world_pose.position.y:.3f} (m)\n'
            output += f'    z: {part_world_pose.position.z:.3f} (m)\n'

            roll, pitch, yaw = rpy_from_quaternion(part_world_pose.orientation)
            output += '  Orientation:\n'
            output += f'    roll: {rad_to_deg_str(roll)}\n'
            output += f'    pitch: {rad_to_deg_str(pitch)}\n'
            output += f'    yaw: {rad_to_deg_str(yaw)}\n'

            output += '==========================\n\n'

        return output
    
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
        image = self.camera_image
        self.get_logger().info(f'Part Count: {self.part_count}', throttle_duration_sec=2.0)

        if image is not None:
            self.get_logger().info(self.parse_advanced_camera_image(image), throttle_duration_sec=5.0)
        # rate = CompetitionInterface.create_rate(2,2.5)
        # rate.sleep()    
        self.collecting_parts()
        
        while (self.competition_state != CompetitionState.ORDER_ANNOUNCEMENTS_DONE):
            try:
                rclpy.spin_once(self)
                self.comparing_orders_parts()  
            except KeyboardInterrupt:
                return
            
        self.get_logger().info('Start done')
        # self.collecting_parts()   
        # self.comparing_orders_parts()
        # self.start_client()

    def breakbeam0_cb(self, msg: BreakBeamStatus):
        if not self.object_detected and msg.object_detected:
            self.part_count += 1

        self.object_detected = msg.object_detected
        
    def _advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
        '''Callback for the topic /ariac/sensors/advanced_camera_0/image

        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''
        self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                        msg.tray_poses,
                                                        msg.sensor_pose)