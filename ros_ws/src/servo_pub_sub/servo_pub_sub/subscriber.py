import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time


from std_msgs.msg import Float32

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)

servo = GPIO.PWM(11, 50)

servo.start(0)
time.sleep(1)

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)

servo = GPIO.PWM(11, 50)


class NumberSubscriber(Node):

    def __init__(self):
        super().__init__("number_subscriber")
        self.subscription = self.create_subscription(
            Float32, "float_topic", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        servo.ChangeDutyCycle(int(msg.data))
        self.get_logger().info(
            "received: %f has data type %s" % (msg.data, type(msg.data))
        )


def main(args=None):
    rclpy.init(args=args)

    number_subscriber = NumberSubscriber()

    rclpy.spin(number_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    number_subscriber.destroy_node()
    rclpy.shutdown()
    servo.stop()
    GPIO.cleanup()
    print("done")


if __name__ == "__main__":
    main()
