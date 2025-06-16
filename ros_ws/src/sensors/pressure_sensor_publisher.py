import rclpy
from rclpy.node import Node

import serial

from std_msgs.msg import Int16MultiArray


serial_device = serial.Serial('/dev/ttyACM0') 

serial_device.baudrate = 9600
serial_device.bytesize = 8
serial_device.parity = 'N'
serial_device.stopbits = 1

class PressureSensorPublisher(Node):
    def __init__(self):
        super().__init__('pressure_sensor_publisher')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'pressure_sensor', 4)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        received_string = serial_device.readline()
        try:
            decoded = [int(i) for i in received_string.split()]
            msg = Int16MultiArray()
            msg.data = decoded
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        except ValueError:
            return 1



def main(args=None):
    rclpy.init(args=args)
    print(args)
    pressure_sensor_publisher = PressureSensorPublisher()
    rclpy.spin(pressure_sensor_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_device.close()
    pressure_sensor_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()