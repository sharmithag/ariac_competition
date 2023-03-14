import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Order
from std_msgs.msg import String


class OrderSubscriber(Node):

    def __init__(self):
        super().__init__('order_subscriber')
        self.subscription = self.create_subscription(
            Order,
            '/ariac/orders',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.id)




if __name__ == '__main__':
    main()

