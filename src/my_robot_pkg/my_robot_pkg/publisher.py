import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    def __init__(self):
        super().__init__('my_publisher')

        self.publisher = self.create_publisher(
            String,
            'my_topic',
            10
        )
        self.timer = self.create_timer(
            1.0,
            self.publish_message
        )
        self.count = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello ROS 2 Jazzy #{self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Publishing: "{msg.data}"'
        )

        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


