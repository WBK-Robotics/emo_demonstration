from rclpy.node import Node
import rclpy
from ur_msgs.srv import SetIO
from time import sleep

class IOServiceClient(Node):

    def __init__(self):
        super().__init__('io_service_client')
        self.client = self.create_client(SetIO, 'set_io')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetIO.Request()

    def set_digital_out(self, pin, value):
        self.req.fun = self.req.FUN_SET_DIGITAL_OUT
        self.req.pin = pin
        self.req.state = value
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info(
                'Result %s' %
                (response,))


def main(args=None):
    rclpy.init()
    node = IOServiceClient()

    # loop through all digital outputs:
    for i in range(0, 8):
        node.set_digital_out(i, True)
        sleep(1)
        node.set_digital_out(i, False)
        sleep(1)
     