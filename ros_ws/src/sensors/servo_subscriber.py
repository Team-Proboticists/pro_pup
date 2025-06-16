import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8MultiArray

import time
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()

# 60 kg-cm ABCD

# mid_points = [325, 342, 306, 301]
# 50: 1Lblue-410, 2L-420, 1R-220, 2R-250
# 105: 1Lblue-280, 2L-280, 1R-350, 2R-400
pulse_50 = [300,300,300,300]
pulse_105 = [300,300,300,300]
# servo_min = [i - 200 for i in mid_points]
# servo_max = [i + 200 for i in mid_points]


pwm.set_pwm_freq(50)

def set_angle(channel, angle):
    pulse = (angle - 50) * (pulse_105[channel] - pulse_50[channel]) / 55 + pulse_50[channel]
    pwm.set_pwm(channel, 0, pulse)
    

class ServoSubscriber(Node):
    def __init__(self):
        super().__init__('servo_subscriber')
        print("started")
        self.subscription = self.create_subscription(
            Int8MultiArray,
            'servos',
            self.listener_callback,
            4)

    def listener_callback(self, msg):
        data = msg.data
        print(data)
        if len(data) == 16:
            for i, j in enumerate(data):
                set_angle(i, 0, j)
        self.get_logger().info('servos moved: "%s"' % msg.data)
        time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)

    servo_subscriber = ServoSubscriber()

    rclpy.spin(servo_subscriber)

    servo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
