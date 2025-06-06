import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from rclpy.executors import ExternalShutdownException

class CounterSubscriber(Node):
    def __init__(self):
        super().__init__('counter_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'counter_topic',
            self.listener_callback,
            10)

        self.declare_parameter('reset_arg', 50)
        self.reset_arg = self.get_parameter('reset_arg').get_parameter_value().integer_value
        self.call_pending = False  # evita múltiples llamadas seguidas

        self.client = self.create_client(Trigger, 'reset_counter')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for reset_counter service...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

        if msg.data >= self.reset_arg and not self.call_pending:
            req = Trigger.Request()
            future = self.client.call_async(req)
            future.add_done_callback(self.service_response_callback)
            self.call_pending = True

    def service_response_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Counter reset successfully via service.')
            else:
                self.get_logger().error('Service responded but failed to reset counter.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        finally:
            self.call_pending = False  # permite futuras llamadas

def main(args=None):
    rclpy.init(args=args)
    node = CounterSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
