import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPub(Node):
    def __init__(self):
        super().__init__('simple_pub_node')
        self.publisher = self.create_publisher(String, '/talk', 1)

    def step(self):
        msg = String()
        msg.data = "Hello World"
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    pub_node = MyPub();

    loop_rate = pub_node.create_rate(2)   #2Hz
    try:
        while(rclpy.ok()):
            rclpy.spin_once(pub_node)
            pub_node.step()
            loop_rate.sleep()

        rclpy.shutdown()
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
