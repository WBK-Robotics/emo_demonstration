from rclpy.node import Node
import rclpy
from ur_msgs.srv import SetIO
from time import sleep

class IOServiceClient(Node):

    def __init__(self):
        super().__init__('io_service_client')
        self.client = self.create_client(SetIO, 'io_and_status_controller/set_io')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetIO.Request()

    def set_digital_out(self, pin, value):
        self.req.fun = self.req.FUN_SET_DIGITAL_OUT
        self.req.pin = pin
        self.req.state = value
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.callback)

    def open_gripper(self):
        self.set_digital_out(16,0.0)
        sleep(0.025)
        self.set_digital_out(17,1.0)


    def close_gripper(self):
        self.set_digital_out(17,0.0)
        sleep(0.025)
        self.set_digital_out(16,1.0)



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

    while True:
        node.open_gripper()
        sleep(1)
        node.close_gripper()
        sleep(1)