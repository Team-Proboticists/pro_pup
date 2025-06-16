import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class NumberPublisher(Node):

    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Float32, 'float_topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = float(input('enter a float: '))
        self.publisher_.publish(msg)
        self.get_logger().info('sent %f' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    number_publisher = NumberPublisher()
    rclpy.spin(number_publisher)

    number_publisher.destroy_node()
    rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    

if __name__ == '__main__':
    main()